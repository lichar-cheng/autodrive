package com.autodrive.slamexport;

public final class ExportArtifacts {
    private final PgmMetadata pgmMetadata;
    private final String yamlText;
    private final String jsonText;
    private final byte[] pcdBytes;

    public ExportArtifacts(PgmMetadata pgmMetadata, String yamlText, String jsonText) {
        this(pgmMetadata, yamlText, jsonText, null);
    }

    public ExportArtifacts(PgmMetadata pgmMetadata, String yamlText, String jsonText, byte[] pcdBytes) {
        this.pgmMetadata = pgmMetadata;
        this.yamlText = yamlText;
        this.jsonText = jsonText;
        this.pcdBytes = pcdBytes == null ? null : pcdBytes.clone();
    }

    public PgmMetadata getPgmMetadata() {
        return pgmMetadata;
    }

    public String getPgmText() {
        return pgmMetadata.getPgmText();
    }

    public String getYamlText() {
        return yamlText;
    }

    public String getJsonText() {
        return jsonText;
    }

    public byte[] getPcdBytes() {
        return pcdBytes == null ? null : pcdBytes.clone();
    }
}
