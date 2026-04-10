package com.autodrive.slamexport;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;

public final class SlamExportTool {
    private static final DecimalFormat THREE_DECIMAL = new DecimalFormat("0.000", DecimalFormatSymbols.getInstance(Locale.US));

    private SlamExportTool() {
    }

    public static LoadedSlam load(Path path) throws IOException {
        byte[] manifestBytes = null;
        byte[] radarBytes = null;
        try (ZipInputStream zis = new ZipInputStream(Files.newInputStream(path))) {
            ZipEntry entry;
            while ((entry = zis.getNextEntry()) != null) {
                if ("manifest.json".equals(entry.getName())) {
                    manifestBytes = readAllBytes(zis);
                } else if ("radar_points.bin".equals(entry.getName())) {
                    radarBytes = readAllBytes(zis);
                }
            }
        }
        if (manifestBytes == null || radarBytes == null) {
            throw new IOException(".slam archive must contain manifest.json and radar_points.bin");
        }
        return new LoadedSlam(parseManifest(new String(manifestBytes, StandardCharsets.UTF_8)), parseRadarPoints(radarBytes));
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> parseManifest(String jsonText) throws IOException {
        ScriptEngine engine = new ScriptEngineManager().getEngineByName("javascript");
        if (engine == null) {
            throw new IOException("JavaScript engine unavailable for JSON parsing");
        }
        try {
            Object parsed = engine.eval("Java.asJSONCompatible(" + jsonText + ")");
            if (!(parsed instanceof Map)) {
                throw new IOException("manifest.json must decode to an object");
            }
            return (Map<String, Object>) deepCopy(parsed);
        } catch (ScriptException exc) {
            throw new IOException("Failed to parse manifest.json", exc);
        }
    }

    public static List<SlamPoint> parseRadarPoints(byte[] data) {
        List<SlamPoint> points = new ArrayList<SlamPoint>();
        ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        while (buffer.remaining() >= 12) {
            points.add(new SlamPoint(buffer.getFloat(), buffer.getFloat(), buffer.getFloat()));
        }
        return points;
    }

    public static ExportArtifacts buildExports(String sourceFile, Map<String, Object> manifest, List<SlamPoint> points, double resolution, int paddingCells) {
        PgmMetadata pgm = buildPgm(sourceFile, manifest, points, resolution, paddingCells);
        String yaml = buildYamlText(sourceFile, resolution, pgm.getOrigin());
        Map<String, Object> exportManifest = deepCopyMap(manifest);
        exportManifest.remove("browser_occupancy");
        exportManifest.remove("trajectory");

        Map<String, Object> mapYaml = new LinkedHashMap<String, Object>();
        mapYaml.put("image", replaceExtension(sourceFile, ".pgm"));
        mapYaml.put("mode", "trinary");
        mapYaml.put("resolution", resolution);
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

        return new ExportArtifacts(pgm, yaml, toJson(exportJson, 0));
    }

    public static ExportArtifacts export(Path slamPath, Path outputDir, double resolution, int paddingCells) throws IOException {
        LoadedSlam loaded = load(slamPath);
        ExportArtifacts artifacts = buildExports(slamPath.getFileName().toString(), loaded.getManifest(), loaded.getRadarPoints(), resolution, paddingCells);
        Files.createDirectories(outputDir);
        String stem = baseName(slamPath.getFileName().toString());
        Files.writeString(outputDir.resolve(stem + ".pgm"), artifacts.getPgmText(), StandardCharsets.UTF_8);
        Files.writeString(outputDir.resolve(stem + ".yaml"), artifacts.getYamlText(), StandardCharsets.UTF_8);
        Files.writeString(outputDir.resolve(stem + ".json"), artifacts.getJsonText(), StandardCharsets.UTF_8);
        return artifacts;
    }

    @SuppressWarnings("unchecked")
    private static PgmMetadata buildPgm(String sourceFile, Map<String, Object> manifest, List<SlamPoint> points, double resolution, int paddingCells) {
        Map<String, Object> browser = manifest.get("browser_occupancy") instanceof Map
            ? (Map<String, Object>) manifest.get("browser_occupancy")
            : null;
        double occupancyVoxel = Math.max(0.02, getDouble(browser, "voxel_size", resolution));
        List<Object> occupiedCells = browser != null && browser.get("occupied_cells") instanceof List
            ? (List<Object>) browser.get("occupied_cells")
            : null;

        long minCellX = Long.MAX_VALUE;
        long maxCellX = Long.MIN_VALUE;
        long minCellY = Long.MAX_VALUE;
        long maxCellY = Long.MIN_VALUE;
        Set<String> occupied = new LinkedHashSet<String>();

        if (occupiedCells != null && !occupiedCells.isEmpty()) {
            for (Object item : occupiedCells) {
                Map<String, Object> cell = (Map<String, Object>) item;
                long ix = Math.round(getDouble(cell, "ix", 0.0));
                long iy = Math.round(getDouble(cell, "iy", 0.0));
                minCellX = Math.min(minCellX, ix);
                maxCellX = Math.max(maxCellX, ix);
                minCellY = Math.min(minCellY, iy);
                maxCellY = Math.max(maxCellY, iy);
                occupied.add(ix + ":" + iy);
            }
        } else {
            if (points.isEmpty()) {
                throw new IllegalArgumentException("No radar points in SLAM");
            }
            for (SlamPoint point : points) {
                long ix = Math.round(point.getX() / resolution);
                long iy = Math.round(point.getY() / resolution);
                minCellX = Math.min(minCellX, ix);
                maxCellX = Math.max(maxCellX, ix);
                minCellY = Math.min(minCellY, iy);
                maxCellY = Math.max(maxCellY, iy);
                occupied.add(ix + ":" + iy);
            }
        }

        long paddedMinX = minCellX - paddingCells;
        long paddedMinY = minCellY - paddingCells;
        long paddedMaxX = maxCellX + paddingCells;
        long paddedMaxY = maxCellY + paddingCells;
        int width = Math.max(1, (int) (paddedMaxX - paddedMinX + 1));
        int height = Math.max(1, (int) (paddedMaxY - paddedMinY + 1));
        int[] grid = new int[width * height];
        for (int i = 0; i < grid.length; i++) {
            grid[i] = 205;
        }

        for (String cellKey : occupied) {
            String[] parts = cellKey.split(":");
            int ix = (int) (Long.parseLong(parts[0]) - paddedMinX);
            int iy = (int) (Long.parseLong(parts[1]) - paddedMinY);
            int flippedY = height - 1 - iy;
            grid[flippedY * width + ix] = 0;
        }

        StringBuilder rows = new StringBuilder();
        rows.append("P2\n# Generated from SLAM occupancy\n");
        rows.append(width).append(" ").append(height).append("\n255\n");
        for (int row = 0; row < height; row++) {
            if (row > 0) {
                rows.append("\n");
            }
            int start = row * width;
            for (int col = 0; col < width; col++) {
                if (col > 0) {
                    rows.append(" ");
                }
                rows.append(grid[start + col]);
            }
        }
        rows.append("\n");

        List<Double> origin = List.of(round3(paddedMinX * occupancyVoxel), round3(paddedMinY * occupancyVoxel), 0.0);
        Map<String, Double> bounds = new LinkedHashMap<String, Double>();
        bounds.put("minX", round3(minCellX * occupancyVoxel));
        bounds.put("maxX", round3(maxCellX * occupancyVoxel));
        bounds.put("minY", round3(minCellY * occupancyVoxel));
        bounds.put("maxY", round3(maxCellY * occupancyVoxel));
        return new PgmMetadata(rows.toString(), width, height, origin, occupied.size(), bounds);
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

    private static String replaceExtension(String fileName, String extension) {
        return baseName(fileName) + extension;
    }

    private static String baseName(String fileName) {
        int index = fileName.lastIndexOf('.');
        return index >= 0 ? fileName.substring(0, index) : fileName;
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
        return " ".repeat(Math.max(0, count));
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
}
