package com.autodrive.slamexport;

public final class ExportArtifacts {
    private final PgmMetadata pgmMetadata;
    private final String yamlText;
    private final String jsonText;

    public ExportArtifacts(PgmMetadata pgmMetadata, String yamlText, String jsonText) {
        this.pgmMetadata = pgmMetadata;
        this.yamlText = yamlText;
        this.jsonText = jsonText;
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
}
