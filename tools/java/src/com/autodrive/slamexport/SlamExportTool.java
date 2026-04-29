package com.autodrive.slamexport;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

public final class SlamExportTool {
    private static final DecimalFormat THREE_DECIMAL = new DecimalFormat("0.000", DecimalFormatSymbols.getInstance(Locale.US));

    private SlamExportTool() {
    }

    public static LoadedSlam load(Path path) throws IOException {
        byte[] manifestBytes = null;
        byte[] gridBytes = null;
        LinkedHashMap<String, byte[]> archiveEntries = new LinkedHashMap<String, byte[]>();
        try (ZipInputStream zis = new ZipInputStream(Files.newInputStream(path))) {
            ZipEntry entry;
            while ((entry = zis.getNextEntry()) != null) {
                byte[] entryBytes = readAllBytes(zis);
                archiveEntries.put(entry.getName(), entryBytes);
                if ("manifest.json".equals(entry.getName())) {
                    manifestBytes = entryBytes;
                } else if ("grid.bin".equals(entry.getName())) {
                    gridBytes = entryBytes;
                }
            }
        }
        if (manifestBytes == null || gridBytes == null) {
            throw new IOException(".slam archive must contain manifest.json and grid.bin");
        }
        Map<String, Object> manifest = parseManifest(new String(manifestBytes, StandardCharsets.UTF_8));
        byte[] pcdBytes = attachPcdPayload(manifest, archiveEntries);
        Map<String, Object> occupancyGrid = buildOccupancyGrid(manifest, gridBytes);
        return new LoadedSlam(manifest, occupancyGrid, pcdBytes);
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> parseManifest(String jsonText) throws IOException {
        try {
            Object parsed = new JsonParser(jsonText).parse();
            if (!(parsed instanceof Map)) {
                throw new IOException("manifest.json must decode to an object");
            }
            return (Map<String, Object>) deepCopy(parsed);
        } catch (RuntimeException exc) {
            throw new IOException("Failed to parse manifest.json", exc);
        }
    }

    public static ExportArtifacts buildExports(String sourceFile, Map<String, Object> manifest, Map<String, Object> occupancyGrid, double resolution, int paddingCells) {
        return buildExports(sourceFile, manifest, occupancyGrid, null, resolution, paddingCells);
    }

    @SuppressWarnings("unchecked")
    public static ExportArtifacts buildExports(
        String sourceFile,
        Map<String, Object> manifest,
        Map<String, Object> occupancyGrid,
        byte[] pcdBytes,
        double resolution,
        int paddingCells
    ) {
        PgmMetadata pgm = buildPgm(occupancyGrid, resolution, paddingCells);
        String yaml = buildYamlText(sourceFile, getDouble(occupancyGrid, "resolution", resolution), pgm.getOrigin());
        Map<String, Object> exportManifest = deepCopyMap(manifest);
        byte[] exportPcdBytes = pcdBytes;
        Object rawPcdMeta = exportManifest.get("pcd_file");
        if (rawPcdMeta instanceof Map) {
            Map<String, Object> pcdMeta = (Map<String, Object>) rawPcdMeta;
            if (Boolean.TRUE.equals(pcdMeta.get("included"))) {
                if (exportPcdBytes == null && pcdMeta.get("content") instanceof byte[]) {
                    exportPcdBytes = ((byte[]) pcdMeta.get("content")).clone();
                }
                Map<String, Object> sanitizedPcdMeta = new LinkedHashMap<String, Object>();
                sanitizedPcdMeta.put("name", replaceExtension(sourceFile, ".pcd"));
                sanitizedPcdMeta.put("included", Boolean.TRUE);
                exportManifest.put("pcd_file", sanitizedPcdMeta);
            }
        }

        Map<String, Object> mapYaml = new LinkedHashMap<String, Object>();
        mapYaml.put("image", replaceExtension(sourceFile, ".pgm"));
        mapYaml.put("mode", "trinary");
        mapYaml.put("resolution", getDouble(occupancyGrid, "resolution", resolution));
        mapYaml.put("origin", pgm.getOrigin());
        mapYaml.put("negate", 0);
        mapYaml.put("occupied_thresh", 0.65);
        mapYaml.put("free_thresh", 0.196);

        Map<String, Object> pgmMeta = new LinkedHashMap<String, Object>();
        pgmMeta.put("width", pgm.getWidth());
        pgmMeta.put("height", pgm.getHeight());
        pgmMeta.put("occupied_cells", pgm.getOccupiedCells());
        pgmMeta.put("bounds", pgm.getBounds());

        Map<String, Object> exportJson = new LinkedHashMap<String, Object>();
        exportJson.put("source_file", sourceFile);
        exportJson.put("map_yaml", mapYaml);
        exportJson.put("pgm_meta", pgmMeta);
        exportJson.put("manifest", exportManifest);

        return new ExportArtifacts(pgm, yaml, toJson(exportJson, 0), exportPcdBytes);
    }

    public static ExportArtifacts export(Path slamPath, Path outputDir, double resolution, int paddingCells) throws IOException {
        LoadedSlam loaded = load(slamPath);
        ExportArtifacts artifacts = buildExports(
            slamPath.getFileName().toString(),
            loaded.getManifest(),
            loaded.getOccupancyGrid(),
            loaded.getPcdContent(),
            resolution,
            paddingCells
        );
        Files.createDirectories(outputDir);
        String stem = baseName(slamPath.getFileName().toString());
        writeUtf8(outputDir.resolve(stem + ".pgm"), artifacts.getPgmText());
        writeUtf8(outputDir.resolve(stem + ".yaml"), artifacts.getYamlText());
        writeUtf8(outputDir.resolve(stem + ".json"), artifacts.getJsonText());
        if (artifacts.getPcdBytes() != null) {
            Files.write(outputDir.resolve(stem + ".pcd"), artifacts.getPcdBytes());
        }
        return artifacts;
    }

    @SuppressWarnings("unchecked")
    private static PgmMetadata buildPgm(Map<String, Object> occupancyGrid, double resolution, int paddingCells) {
        paddingCells = 0;
        int width = (int) Math.round(getDouble(occupancyGrid, "width", 0.0));
        int height = (int) Math.round(getDouble(occupancyGrid, "height", 0.0));
        double gridResolution = Math.max(0.02, getDouble(occupancyGrid, "resolution", resolution));
        List<Object> data = occupancyGrid.get("data") instanceof List
            ? (List<Object>) occupancyGrid.get("data")
            : new ArrayList<Object>();
        if (width <= 0 || height <= 0 || data.size() != width * height) {
            throw new IllegalArgumentException("No occupancy grid in SLAM");
        }

        StringBuilder rows = new StringBuilder();
        rows.append("P2\n# Generated from SLAM occupancy\n");
        rows.append(width).append(" ").append(height).append("\n255\n");
        for (int row = 0; row < height; row++) {
            if (row > 0) {
                rows.append("\n");
            }
            int start = (height - 1 - row) * width;
            for (int col = 0; col < width; col++) {
                if (col > 0) {
                    rows.append(" ");
                }
                int value = getInt(data.get(start + col), -1);
                rows.append(value >= 50 ? 0 : value == 0 ? 254 : 205);
            }
        }
        rows.append("\n");

        Map<String, Object> originMeta = occupancyGrid.get("origin") instanceof Map
            ? (Map<String, Object>) occupancyGrid.get("origin")
            : new LinkedHashMap<String, Object>();
        double originX = getDouble(originMeta, "x", 0.0);
        double originY = getDouble(originMeta, "y", 0.0);
        int occupiedCount = 0;
        for (Object value : data) {
            if (getInt(value, -1) >= 50) {
                occupiedCount += 1;
            }
        }
        List<Double> origin = new ArrayList<Double>();
        origin.add(round3(originX));
        origin.add(round3(originY));
        origin.add(0.0);
        Map<String, Double> bounds = new LinkedHashMap<String, Double>();
        bounds.put("minX", round3(originX));
        bounds.put("maxX", round3(originX + width * gridResolution));
        bounds.put("minY", round3(originY));
        bounds.put("maxY", round3(originY + height * gridResolution));
        return new PgmMetadata(rows.toString(), width, height, origin, occupiedCount, bounds);
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> buildOccupancyGrid(Map<String, Object> manifest, byte[] gridBytes) {
        Map<String, Object> occupancyGrid = manifest.get("occupancy_grid") instanceof Map
            ? deepCopyMap((Map<String, Object>) manifest.get("occupancy_grid"))
            : new LinkedHashMap<String, Object>();
        List<Integer> data = new ArrayList<Integer>();
        for (byte value : gridBytes) {
            data.add((int) value);
        }
        occupancyGrid.put("data", data);
        return occupancyGrid;
    }

    private static String buildYamlText(String fileName, double resolution, List<Double> origin) {
        return "image: " + replaceExtension(fileName, ".pgm") + "\n"
            + "mode: trinary\n"
            + "resolution: " + THREE_DECIMAL.format(resolution) + "\n"
            + "origin: [" + THREE_DECIMAL.format(origin.get(0)) + ", " + THREE_DECIMAL.format(origin.get(1)) + ", " + Math.round(origin.get(2)) + "]\n"
            + "negate: 0\n"
            + "occupied_thresh: 0.65\n"
            + "free_thresh: 0.196";
    }

    private static byte[] readAllBytes(ZipInputStream zis) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buffer = new byte[4096];
        int read;
        while ((read = zis.read(buffer)) >= 0) {
            out.write(buffer, 0, read);
        }
        return out.toByteArray();
    }

    @SuppressWarnings("unchecked")
    private static byte[] attachPcdPayload(Map<String, Object> manifest, Map<String, byte[]> archiveEntries) {
        Map<String, Object> pcdMeta = readPcdMeta(manifest.get("pcd_file"), "name");
        if (pcdMeta == null) {
            pcdMeta = readPcdMeta(manifest.get("pcd"), "file");
        }
        if (pcdMeta == null) {
            return null;
        }
        String pcdName = String.valueOf(pcdMeta.get("resolved_name"));
        byte[] pcdBytes = archiveEntries.get(pcdName);
        if (pcdBytes == null) {
            throw new IllegalArgumentException("PCD entry missing from SLAM archive: " + pcdName);
        }
        Map<String, Object> manifestPcdMeta = new LinkedHashMap<String, Object>();
        manifestPcdMeta.put("name", pcdName);
        manifestPcdMeta.put("included", Boolean.TRUE);
        manifestPcdMeta.put("content", pcdBytes.clone());
        manifest.put("pcd_file", manifestPcdMeta);
        return pcdBytes.clone();
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> readPcdMeta(Object rawPcdMeta, String nameKey) {
        if (!(rawPcdMeta instanceof Map)) {
            return null;
        }
        Map<String, Object> pcdMeta = (Map<String, Object>) rawPcdMeta;
        if (!Boolean.TRUE.equals(pcdMeta.get("included"))) {
            return null;
        }
        Map<String, Object> normalized = new LinkedHashMap<String, Object>(pcdMeta);
        normalized.put("resolved_name", String.valueOf(pcdMeta.get(nameKey) == null ? "map.pcd" : pcdMeta.get(nameKey)));
        return normalized;
    }

    private static String replaceExtension(String fileName, String extension) {
        return baseName(fileName) + extension;
    }

    private static String baseName(String fileName) {
        int index = fileName.lastIndexOf('.');
        return index >= 0 ? fileName.substring(0, index) : fileName;
    }

    private static void writeUtf8(Path path, String text) throws IOException {
        Files.write(path, text.getBytes(StandardCharsets.UTF_8));
    }

    private static double getDouble(Map<String, Object> map, String key, double defaultValue) {
        if (map == null) {
            return defaultValue;
        }
        Object value = map.get(key);
        if (value instanceof Number) {
            return ((Number) value).doubleValue();
        }
        if (value instanceof String) {
            try {
                return Double.parseDouble((String) value);
            } catch (NumberFormatException ignored) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    private static int getInt(Object value, int defaultValue) {
        if (value instanceof Number) {
            return ((Number) value).intValue();
        }
        if (value instanceof String) {
            try {
                return Integer.parseInt((String) value);
            } catch (NumberFormatException ignored) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    @SuppressWarnings("unchecked")
    private static Object deepCopy(Object value) {
        if (value instanceof Map) {
            Map<String, Object> copy = new LinkedHashMap<String, Object>();
            for (Map.Entry<String, Object> entry : ((Map<String, Object>) value).entrySet()) {
                copy.put(entry.getKey(), deepCopy(entry.getValue()));
            }
            return copy;
        }
        if (value instanceof List) {
            List<Object> copy = new ArrayList<Object>();
            for (Object item : (List<Object>) value) {
                copy.add(deepCopy(item));
            }
            return copy;
        }
        return value;
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> deepCopyMap(Map<String, Object> value) {
        return (Map<String, Object>) deepCopy(value);
    }

    @SuppressWarnings("unchecked")
    private static String toJson(Object value, int indent) {
        if (value == null) {
            return "null";
        }
        if (value instanceof String) {
            return "\"" + escapeJson((String) value) + "\"";
        }
        if (value instanceof Number || value instanceof Boolean) {
            return String.valueOf(value);
        }
        if (value instanceof Map) {
            StringBuilder builder = new StringBuilder();
            builder.append("{");
            boolean first = true;
            for (Map.Entry<String, Object> entry : ((Map<String, Object>) value).entrySet()) {
                if (!first) {
                    builder.append(",");
                }
                builder.append("\n").append(spaces(indent + 2));
                builder.append("\"").append(escapeJson(entry.getKey())).append("\": ");
                builder.append(toJson(entry.getValue(), indent + 2));
                first = false;
            }
            if (!first) {
                builder.append("\n").append(spaces(indent));
            }
            builder.append("}");
            return builder.toString();
        }
        if (value instanceof List) {
            List<Object> list = (List<Object>) value;
            StringBuilder builder = new StringBuilder();
            builder.append("[");
            for (int i = 0; i < list.size(); i++) {
                if (i > 0) {
                    builder.append(", ");
                }
                builder.append(toJson(list.get(i), indent));
            }
            builder.append("]");
            return builder.toString();
        }
        return "\"" + escapeJson(String.valueOf(value)) + "\"";
    }

    private static String spaces(int count) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < Math.max(0, count); i++) {
            builder.append(' ');
        }
        return builder.toString();
    }

    private static String escapeJson(String text) {
        return text
            .replace("\\", "\\\\")
            .replace("\"", "\\\"")
            .replace("\n", "\\n")
            .replace("\r", "\\r")
            .replace("\t", "\\t");
    }

    private static double round3(double value) {
        return Double.parseDouble(THREE_DECIMAL.format(value));
    }

    private static final class JsonParser {
        private final String text;
        private int index = 0;

        JsonParser(String text) {
            this.text = text;
        }

        Object parse() {
            Object value = parseValue();
            skipWhitespace();
            if (index != text.length()) {
                throw new IllegalArgumentException("Unexpected trailing JSON content");
            }
            return value;
        }

        private Object parseValue() {
            skipWhitespace();
            if (index >= text.length()) {
                throw new IllegalArgumentException("Unexpected end of JSON");
            }
            char ch = text.charAt(index);
            if (ch == '{') {
                return parseObject();
            }
            if (ch == '[') {
                return parseArray();
            }
            if (ch == '"') {
                return parseString();
            }
            if (ch == 't') {
                expect("true");
                return Boolean.TRUE;
            }
            if (ch == 'f') {
                expect("false");
                return Boolean.FALSE;
            }
            if (ch == 'n') {
                expect("null");
                return null;
            }
            return parseNumber();
        }

        private Map<String, Object> parseObject() {
            LinkedHashMap<String, Object> map = new LinkedHashMap<String, Object>();
            expect("{");
            skipWhitespace();
            if (peek('}')) {
                index += 1;
                return map;
            }
            while (true) {
                String key = parseString();
                skipWhitespace();
                expect(":");
                map.put(key, parseValue());
                skipWhitespace();
                if (peek('}')) {
                    index += 1;
                    return map;
                }
                expect(",");
            }
        }

        private List<Object> parseArray() {
            List<Object> list = new ArrayList<Object>();
            expect("[");
            skipWhitespace();
            if (peek(']')) {
                index += 1;
                return list;
            }
            while (true) {
                list.add(parseValue());
                skipWhitespace();
                if (peek(']')) {
                    index += 1;
                    return list;
                }
                expect(",");
            }
        }

        private String parseString() {
            expect("\"");
            StringBuilder builder = new StringBuilder();
            while (index < text.length()) {
                char ch = text.charAt(index++);
                if (ch == '"') {
                    return builder.toString();
                }
                if (ch != '\\') {
                    builder.append(ch);
                    continue;
                }
                if (index >= text.length()) {
                    throw new IllegalArgumentException("Invalid JSON escape");
                }
                char escaped = text.charAt(index++);
                switch (escaped) {
                    case '"':
                    case '\\':
                    case '/':
                        builder.append(escaped);
                        break;
                    case 'b':
                        builder.append('\b');
                        break;
                    case 'f':
                        builder.append('\f');
                        break;
                    case 'n':
                        builder.append('\n');
                        break;
                    case 'r':
                        builder.append('\r');
                        break;
                    case 't':
                        builder.append('\t');
                        break;
                    case 'u':
                        if (index + 4 > text.length()) {
                            throw new IllegalArgumentException("Invalid unicode escape");
                        }
                        builder.append((char) Integer.parseInt(text.substring(index, index + 4), 16));
                        index += 4;
                        break;
                    default:
                        throw new IllegalArgumentException("Unsupported JSON escape: \\" + escaped);
                }
            }
            throw new IllegalArgumentException("Unterminated JSON string");
        }

        private Number parseNumber() {
            int start = index;
            if (peek('-')) {
                index += 1;
            }
            while (index < text.length() && Character.isDigit(text.charAt(index))) {
                index += 1;
            }
            boolean isFloat = false;
            if (peek('.')) {
                isFloat = true;
                index += 1;
                while (index < text.length() && Character.isDigit(text.charAt(index))) {
                    index += 1;
                }
            }
            if (peek('e') || peek('E')) {
                isFloat = true;
                index += 1;
                if (peek('+') || peek('-')) {
                    index += 1;
                }
                while (index < text.length() && Character.isDigit(text.charAt(index))) {
                    index += 1;
                }
            }
            String raw = text.substring(start, index);
            return isFloat ? Double.parseDouble(raw) : Long.parseLong(raw);
        }

        private void skipWhitespace() {
            while (index < text.length() && Character.isWhitespace(text.charAt(index))) {
                index += 1;
            }
        }

        private boolean peek(char ch) {
            return index < text.length() && text.charAt(index) == ch;
        }

        private void expect(String expected) {
            skipWhitespace();
            if (!text.startsWith(expected, index)) {
                throw new IllegalArgumentException("Expected " + expected);
            }
            index += expected.length();
        }
    }
}
