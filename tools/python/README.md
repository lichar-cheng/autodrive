# Python SLAM Export Tool

这个目录提供一个可直接单文件复用的 Python 类：`SlamExportTool`。

支持能力：

- 读取 `.slam` / `.stcm` 压缩包里的 `manifest.json` 和 `grid.bin`
- 读取可选的内嵌 `pcd`
- 导出 `PGM` / `YAML` / `JSON`，有点云时额外导出同名 `PCD`
- 直接导出到目标目录

不包含能力：

- 不负责导入原生 `pgm + yaml` 地图

最小示例：

```python
from pathlib import Path

from slam_export_tool import SlamExportTool

artifacts = SlamExportTool.export(
    Path("demo.slam"),
    Path("out"),
    resolution=0.2,
    padding_cells=8,
)

print(artifacts.pgm_meta)
```
