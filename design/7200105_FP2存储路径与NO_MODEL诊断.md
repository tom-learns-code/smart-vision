# FP2 7200105 存储路径与 NO_MODEL 诊断

## 现象

二号摄像头以 IDE 调试模式运行 `7200104` 时连续输出：

```text
FP image labels unavailable
FP_IMAGE_FUSE ... status=2 reason=NO_MODEL
FP_IDE_IMAGE ... id=255 status=2
```

`status=2` 表示模型不可用，不是双模型结果冲突，也不是 0.80 阈值过高。脚本在读取图片 labels 之前就失败，因此两个图片模型均未开始加载和推理。

## 现场核对

设备盘符为 `H:`，根目录已经存在并保留以下文件：

- `smartcar_class_model_qat_1.tflite`
- `smartcar_class_model_qat_2.tflite`
- `smartcar_digit_model_qat.tflite`
- `model_labels.txt`
- `digit_labels.txt`
- `main.py`
- `openart_roi_classifier.py`

因此不是电脑侧漏拷模型，更可能是 OpenART 运行方式变化后当前目录或存储挂载点与原脚本假设不一致，或者设备刚写入 FAT 后运行时目录缓存尚未刷新。

## 7200105 修改

- 保留用户 ROI：`(55,40,245,180)`。
- 保留比赛默认：`IDE_DEBUG_ENABLE=False`。
- 保留伪识别默认开启、FP2 协议、双图片模型融合、数字模型和 0.80 阈值。
- 每个模型和 labels 依次搜索 `/sd`、`/flash`、`/sdcard`、根目录和当前目录。
- 文件存在性不再只依赖 `os.stat()`，改为实际执行 `open(path, "rb")` 后关闭。
- IDE 调试打开时输出 `FP_FS`，列出各挂载点目录；输出 `FP_FILE`，给出每个候选路径的命中或失败原因。
- MCU 代码和全局摄像头代码均未修改。

电脑源文件、`H:\main.py` 和 `H:\openart_roi_classifier.py` 的 SHA256 均为：

```text
122014E313F79E7EB81B6B048D1BFD90ACDF265B3B199BA019047FD3E676C300
```

## 下一步验证

1. 停止 IDE 中正在运行的旧脚本，安全弹出或复位二号摄像头一次，让 FAT 目录重新挂载。
2. 在 OpenMV IDE 当前运行脚本中把 `IDE_DEBUG_ENABLE` 临时改为 `True`，保持 `IDE_DEBUG_MODE_IMAGE`。
3. 运行后先确认 `FP_BUILD id=7200105` 和 ROI 为 `(55,40,245,180)`。
4. 保存从 `FP_FS_DIAG begin` 开始，到首个 `FP_IMAGE_FUSE` 和 `FP_IDE_IMAGE` 为止的完整日志。

正常结果应至少出现：

```text
FP_FILE kind=image_labels found=...
FP loading image1 model ...
FP loading image2 model ...
FP image models ready m1=1 m2=1 labels=...
FP_IMAGE_FUSE ... reason=...
FP_IDE_IMAGE ... status=0
```

如果仍失败，`FP_FS` 和 `FP_FILE` 会直接指出实际挂载点、目录内容或打开错误，不再只留下模糊的 `NO_MODEL`。

## 三端状态

- MCU：未修改，`7100959`。
- 全局摄像头：未修改，`7100428`。
- 二号摄像头：已修改并同步，`7200105`。

## 7200105 实机诊断结论

实机返回：

```text
FP_FS root=. unavailable err=[Errno 19] ENODEV
FP_FS root=/ count=0 entries=[]
FP_FS root=/sd unavailable err=[Errno 19] ENODEV
FP_FS root=/flash unavailable err=[Errno 19] ENODEV
FP_FS root=/sdcard unavailable err=[Errno 19] ENODEV
```

这证明问题不再是文件名或候选路径，而是脚本运行时没有任何可用文件系统。Windows 此时仍把同一张 FAT32 SD 卡作为 `OPENMVSD (H:)` 挂载，设备侧无法同时读取。旧版 `/sd/mobilenet.tflite` 曾成功加载，说明模型加载接口和 SD 卡硬件本身此前可用；当前优先恢复正确的存储所有权，不继续改模型、阈值或融合算法。

现场操作顺序更新为：停止脚本 -> Windows 安全弹出 `OPENMVSD (H:)` -> 保持 USB 线连接 -> 复位摄像头 -> IDE 重新运行 7200105。若文件系统恢复，应看到 `FP_FS root=/sdcard` 或其他根目录含模型文件，随后出现 `FP_FILE ... found=...`。

## 延迟挂载复测通过

按弹出 SD 卡并重新启动的流程复测后，启动最早期的 `FP_FS` 仍暂时报 `ENODEV`，但在首个 IDE 图片识别触发时文件系统已经完成挂载，随后成功输出：

```text
FP_FILE kind=image_labels found=/sd/model_labels.txt
FP_FILE kind=image1_model found=/sd/smartcar_class_model_qat_1.tflite
FP loading image1 model to fb: /sd/smartcar_class_model_qat_1.tflite
FP_FILE kind=image2_model found=/sd/smartcar_class_model_qat_2.tflite
FP loading image2 model to fb: /sd/smartcar_class_model_qat_2.tflite
FP image models ready m1=1 m2=1 labels=10
```

因此 SD 卡、labels、两个图片模型、路径搜索和 `tf.load(..., load_to_fb=True)` 均已验证成功。开头位于 `process_pending_request` 的 traceback 是 IDE 主动中断上一轮脚本产生的退出记录，不是新一轮模型加载故障。下一步只采集连续 `FP_IMAGE_FUSE` 与 `FP_IDE_IMAGE`，评估两个模型的分类、0.80 阈值和推理耗时；本阶段不再修改路径。

后续实测没有出现上述融合输出，而是停在双模型加载完成处。该现象已转入`7200106_FP2双模型串行推理与阶段诊断.md`处理：路径结论保留，双图片模型改为逐个加载、推理和释放。
