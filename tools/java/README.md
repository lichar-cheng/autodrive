# Java SLAM Export Tool

This module provides in-memory and file export APIs for `.slam` archives.
It supports exporting `PGM` / `YAML` / `JSON`, and writes `<slam-stem>.pcd`
when the archive contains an embedded point-cloud payload.

## Compile

```bash
cd tools/java
mkdir -p out
javac --release 8 -d out $(find src test -name '*.java')
```

## Run Self-Test

```bash
cd tools/java
mkdir -p out
javac --release 8 -d out $(find src test -name '*.java')
java -cp out com.autodrive.slamexport.SlamExportToolSelfTest
```
