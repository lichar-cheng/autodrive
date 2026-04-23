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
        assertTrue(Files.readString(exportDir.resolve("grid_only.yaml"), StandardCharsets.UTF_8).contains("resolution: 0.100"));
    }

    private static void createSlam(Path path, String manifestJson, byte[] gridBytes) throws IOException {
        try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(path))) {
            zos.putNextEntry(new ZipEntry("manifest.json"));
            zos.write(manifestJson.getBytes(StandardCharsets.UTF_8));
            zos.closeEntry();

            zos.putNextEntry(new ZipEntry("grid.bin"));
            zos.write(gridBytes);
            zos.closeEntry();
        }
    }

    private static void assertTrue(boolean condition) {
        if (!condition) {
            throw new AssertionError("assertion failed");
        }
    }
}
