# Java SLAM Export Tool

This module provides in-memory and file export APIs for `.slam` archives.

## Compile

```bash
cd tools/slam_export/java
mkdir -p out
javac -d out $(find src test -name '*.java')
```

## Run Self-Test

```bash
cd tools/slam_export/java
mkdir -p out
javac -d out $(find src test -name '*.java')
java -cp out com.autodrive.slamexport.SlamExportToolSelfTest
```
