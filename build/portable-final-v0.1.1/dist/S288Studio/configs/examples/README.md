# 空白配置示例

这里的零位、ExPos、验证状态和部署关节映射均为空，不是任何真实机器人的标定结果。

软件在缺少默认配置时会使用空白设置。直接启动即可，首次保存后自动生成`configs/calibration.json`或`configs/demo_calibration.json`。

如需从文件开始，可在界面“加载配置”中选择对应实机/模拟示例；保存时另存到`configs/`，不要覆盖这里的公共模板。

`model_bindings.example.json`与`demo_model_bindings.example.json`是当前机械模型的空绑定格式。分别复制为`configs/model_bindings.json`、`configs/demo_model_bindings.json`后，可在对应模式中逐个绑定。模型指纹用于核对资产，不能证明零位或真实ID正确。

个人配置、外观偏好和`.bak`均由`.gitignore`排除。发布前仍需核对提交清单。
