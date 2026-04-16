package com.autodrive.slamexport;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

public final class SlamExportToolSelfTest {
    public static void main(String[] args) throws Exception {
        testLoadAndBuildFromBrowserOccupancy();
        testFallbackToRadarPointsAndWriteFiles();
        System.out.println("Java self-test passed");
    }

    private static void testLoadAndBuildFromBrowserOccupancy() throws Exception {
        Path tempDir = Files.createTempDirectory("slam-java-test");
        Path slamPath = tempDir.resolve("demo.slam");
        String manifest = "{"
            + "\"version\":\"stcm.v2\","
            + "\"trajectory\":[{\"x\":1}],"
            + "\"browser_occupancy\":{\"voxel_size\":0.2,\"occupied_cells\":[{\"ix\":0,\"iy\":0},{\"ix\":1,\"iy\":0}]},"
            + "\"poi\":[{\"name\":\"A\",\"x\":1.0,\"y\":2.0}]"
            + "}";
        createSlam(slamPath, manifest, List.of(new SlamPoint(0.0f, 0.0f, 1.0f)));

        LoadedSlam loaded = SlamExportTool.load(slamPath);
        ExportArtifacts artifacts = SlamExportTool.buildExports("demo.slam", loaded.getManifest(), loaded.getRadarPoints(), 0.1, 1);

        assertTrue(artifacts.getPgmText().startsWith("P2\n# Generated from SLAM occupancy\n4 3\n255\n"));
        assertTrue(artifacts.getYamlText().contains("image: demo.pgm"));
        assertTrue(artifacts.getYamlText().contains("origin: [-0.200, -0.200, 0]"));
        assertTrue(artifacts.getJsonText().contains("\"source_file\": \"demo.slam\""));
        assertTrue(artifacts.getJsonText().contains("\"occupied_cells\": 2"));
        assertTrue(!artifacts.getJsonText().contains("\"browser_occupancy\""));
        assertTrue(!artifacts.getJsonText().contains("\"trajectory\""));
    }

    private static void testFallbackToRadarPointsAndWriteFiles() throws Exception {
        Path tempDir = Files.createTempDirectory("slam-java-export");
        Path slamPath = tempDir.resolve("points_only.slam");
        String manifest = "{"
            + "\"version\":\"stcm.v2\","
            + "\"poi\":[{\"name\":\"B\",\"x\":0.0,\"y\":0.0}]"
            + "}";
        createSlam(slamPath, manifest, List.of(
            new SlamPoint(0.0f, 0.0f, 1.0f),
            new SlamPoint(0.1f, 0.0f, 1.0f)
        ));

        Path exportDir = tempDir.resolve("out");
        ExportArtifacts artifacts = SlamExportTool.export(slamPath, exportDir, 0.1, 1);

        assertTrue(artifacts.getPgmMetadata().getOccupiedCells() == 2);
        assertTrue(Files.exists(exportDir.resolve("points_only.pgm")));
        assertTrue(Files.exists(exportDir.resolve("points_only.yaml")));
        assertTrue(Files.exists(exportDir.resolve("points_only.json")));
        assertTrue(Files.readString(exportDir.resolve("points_only.yaml"), StandardCharsets.UTF_8).contains("resolution: 0.100"));
    }

    private static void createSlam(Path path, String manifestJson, List<SlamPoint> points) throws IOException {
        try (ZipOutputStream zos = new ZipOutputStream(Files.newOutputStream(path))) {
            zos.putNextEntry(new ZipEntry("manifest.json"));
            zos.write(manifestJson.getBytes(StandardCharsets.UTF_8));
            zos.closeEntry();

            zos.putNextEntry(new ZipEntry("radar_points.bin"));
            zos.write(encodePoints(points));
            zos.closeEntry();
        }
    }

    private static byte[] encodePoints(List<SlamPoint> points) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        ByteBuffer buffer = ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN);
        for (SlamPoint point : points) {
            buffer.clear();
            buffer.putFloat(point.getX());
            buffer.putFloat(point.getY());
            buffer.putFloat(point.getIntensity());
            out.write(buffer.array(), 0, 12);
        }
        return out.toByteArray();
    }

    private static void assertTrue(boolean condition) {
        if (!condition) {
            throw new AssertionError("assertion failed");
        }
    }
}
