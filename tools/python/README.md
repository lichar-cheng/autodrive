# Python SLAM Export Tool

这个目录提供一个给外部复用的 Python 类：`SlamExportTool`。

支持能力：

- 读取 `.slam` / `.stcm` 压缩包里的 `manifest.json` 和 `radar_points.bin`
- 生成 `PGM` / `YAML` / `JSON`
- 直接导出到目标目录

最小示例：

```python
from pathlib import Path

from tool.slam_export_tool import SlamExportTool

artifacts = SlamExportTool.export(
    Path("demo.slam"),
    Path("out"),
    resolution=0.2,
    padding_cells=8,
)

print(artifacts.pgm_meta)
```
