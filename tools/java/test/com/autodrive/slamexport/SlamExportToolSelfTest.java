package com.autodrive.slamexport;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public final class SlamExportToolSelfTest {
    public static void main(String[] args) throws Exception {
        testLoadAndBuildFromOccupancyGrid();
        testExportWritesFilesFromOccupancyGrid();
        testLoadAndExportOptionalPcdWithSameStem();
        testLoadAndExportLegacyDesktopPcdFormat();
        System.out.println("Java self-test passed");
    }

    private static void testLoadAndBuildFromOccupancyGrid() throws Exception {
        Path tempDir = Files.createTempDirectory("slam-java-test");
        Path slamPath = tempDir.resolve("demo.slam");
        String manifest = "{"
            + "\"version\":\"slam.v4\","
            + "\"map_storage\":\"occupancy_grid\","
            + "\"occupancy_grid\":{\"width\":2,\"height\":2,\"resolution\":0.2,\"origin\":{\"x\":0.0,\"y\":0.0},\"encoding\":\"int8\",\"values\":{\"unknown\":-1,\"free\":0,\"occupied\":100}},"
            + "\"poi\":[{\"name\":\"A\",\"x\":1.0,\"y\":2.0}]"
            + "}";
        createSlam(slamPath, manifest, new byte[] {(byte) 100, (byte) 100, 0, (byte) 255});

        LoadedSlam loaded = SlamExportTool.load(slamPath);
        ExportArtifacts artifacts = SlamExportTool.buildExports("demo.slam", loaded.getManifest(), loaded.getOccupancyGrid(), 0.2, 1);

        assertTrue(loaded.getOccupancyGrid().get("data") instanceof List);
        assertTrue(artifacts.getPgmText().startsWith("P2\n# Generated from SLAM occupancy\n2 2\n255\n"));
        assertTrue(artifacts.getYamlText().contains("image: demo.pgm"));
        assertTrue(artifacts.getYamlText().contains("origin: [0.000, 0.000, 0]"));
        assertTrue(artifacts.getJsonText().contains("\"source_file\": \"demo.slam\""));
        assertTrue(artifacts.getJsonText().contains("\"occupied_cells\": 2"));
    }

    private static void testExportWritesFilesFromOccupancyGrid() throws Exception {
        Path tempDir = Files.createTempDirectory("slam-java-export");
        Path slamPath = tempDir.resolve("grid_only.slam");
        String manifest = "{"
            + "\"version\":\"slam.v4\","
            + "\"map_storage\":\"occupancy_grid\","
            + "\"occupancy_grid\":{\"width\":2,\"height\":1,\"resolution\":0.1,\"origin\":{\"x\":0.0,\"y\":0.0},\"encoding\":\"int8\",\"values\":{\"unknown\":-1,\"free\":0,\"occupied\":100}},"
            + "\"poi\":[{\"name\":\"B\",\"x\":0.0,\"y\":0.0}]"
            + "}";
        createSlam(slamPath, manifest, new byte[] {(byte) 100, 0});

        Path exportDir = tempDir.resolve("out");
        ExportArtifacts artifacts = SlamExportTool.export(slamPath, exportDir, 0.1, 1);

        assertTrue(artifacts.getPgmMetadata().getOccupiedCells() == 1);
        assertTrue(Files.exists(exportDir.resolve("grid_only.pgm")));
        assertTrue(Files.exists(exportDir.resolve("grid_only.yaml")));
        assertTrue(Files.exists(exportDir.resolve("grid_only.json")));
        assertTrue(readUtf8(exportDir.resolve("grid_only.yaml")).contains("resolution: 0.100"));
    }

    private static void testLoadAndExportOptionalPcdWithSameStem() throws Exception {
        Path tempDir = Files.createTempDirectory("slam-java-pcd");
        Path slamPath = tempDir.resolve("demo.slam");
        String manifest = "{"
            + "\"version\":\"slam.v4\","
            + "\"map_storage\":\"occupancy_grid\","
            + "\"pcd_file\":{\"name\":\"map.pcd\",\"included\":true},"
            + "\"occupancy_grid\":{\"width\":1,\"height\":1,\"resolution\":0.1,\"origin\":{\"x\":0.0,\"y\":0.0},\"encoding\":\"int8\",\"values\":{\"unknown\":-1,\"free\":0,\"occupied\":100}}"
            + "}";
        createSlam(slamPath, manifest, new byte[] {(byte) 100}, "map.pcd", "pcd-bytes".getBytes(StandardCharsets.UTF_8));

        LoadedSlam loaded = SlamExportTool.load(slamPath);
        assertTrue(loaded.getManifest().containsKey("pcd_file"));
        assertTrue("map.pcd".equals(((java.util.Map<?, ?>) loaded.getManifest().get("pcd_file")).get("name")));
        assertTrue("pcd-bytes".equals(new String(loaded.getPcdContent(), StandardCharsets.UTF_8)));

        Path exportDir = tempDir.resolve("out");
        SlamExportTool.export(slamPath, exportDir, 0.1, 1);

        assertTrue(Files.exists(exportDir.resolve("demo.pcd")));
        assertTrue("pcd-bytes".equals(readUtf8(exportDir.resolve("demo.pcd"))));
    }

    private static void testLoadAndExportLegacyDesktopPcdFormat() throws Exception {
        Path tempDir = Files.createTempDirectory("slam-java-pcd-legacy");
        Path slamPath = tempDir.resolve("desktop_map_pcd.slam");
        String manifest = "{"
            + "\"version\":\"slam.v4\","
            + "\"map_storage\":\"occupancy_grid\","
            + "\"pcd\":{\"included\":true,\"file\":\"scans.pcd\"},"
            + "\"occupancy_grid\":{\"width\":1,\"height\":1,\"resolution\":0.1,\"origin\":{\"x\":0.0,\"y\":0.0},\"encoding\":\"int8\",\"values\":{\"unknown\":-1,\"free\":0,\"occupied\":100}}"
            + "}";
        createSlam(slamPath, manifest, new byte[] {(byte) 100}, "scans.pcd", "pcd-bytes".getBytes(StandardCharsets.UTF_8));

        LoadedSlam loaded = SlamExportTool.load(slamPath);
        assertTrue(loaded.getManifest().containsKey("pcd_file"));
        assertTrue("scans.pcd".equals(((java.util.Map<?, ?>) loaded.getManifest().get("pcd_file")).get("name")));
        assertTrue("pcd-bytes".equals(new String(loaded.getPcdContent(), StandardCharsets.UTF_8)));

        Path exportDir = tempDir.resolve("out");
        SlamExportTool.export(slamPath, exportDir, 0.1, 1);

        assertTrue(Files.exists(exportDir.resolve("desktop_map_pcd.pcd")));
        assertTrue("pcd-bytes".equals(readUtf8(exportDir.resolve("desktop_map_pcd.pcd"))));
    }

    private static void createSlam(Path path, String manifestJson, byte[] gridBytes) throws IOException {
        createSlam(path, manifestJson, gridBytes, null, null);
    }

    private static void createSlam(Path path, String manifestJson, byte[] gridBytes, String pcdName, byte[] pcdBytes) throws IOException {
        try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(path))) {
            zos.putNextEntry(new ZipEntry("manifest.json"));
            zos.write(manifestJson.getBytes(StandardCharsets.UTF_8));
            zos.closeEntry();

            zos.putNextEntry(new ZipEntry("grid.bin"));
            zos.write(gridBytes);
            zos.closeEntry();

            if (pcdName != null && pcdBytes != null) {
                zos.putNextEntry(new ZipEntry(pcdName));
                zos.write(pcdBytes);
                zos.closeEntry();
            }
        }
    }

    private static void assertTrue(boolean condition) {
        if (!condition) {
            throw new AssertionError("assertion failed");
        }
    }

    private static String readUtf8(Path path) throws IOException {
        return new String(Files.readAllBytes(path), StandardCharsets.UTF_8);
    }
}
