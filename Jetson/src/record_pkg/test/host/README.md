# 录制请求回归测试

测试直接编译生产 `control_requests.hpp`。`RecordConsole::start()` 和 `stop()`
在持有会话互斥锁时调用这两个判断，再创建或停止会话。测试覆盖多页面请求、
旧开始/停止请求、当前请求的幂等重试，以及无符号代次溢出。

从仓库根目录执行（CMake 构建目录可自行指定）：

```sh
cmake -S Jetson/src/record_pkg/test/host -B /tmp/gc-record-control-tests
cmake --build /tmp/gc-record-control-tests
ctest --test-dir /tmp/gc-record-control-tests --output-on-failure
```

也可直接使用 C++17 编译器：

```sh
c++ -std=c++17 -Wall -Wextra -Wpedantic -I Jetson/src/record_pkg/src \
  Jetson/src/record_pkg/test/host/test_control_requests.cpp -o /tmp/test_record_control
/tmp/test_record_control
```

此入口不下载依赖，也不需要 ROS、GStreamer、FFmpeg、相机或录像输出目录。
它验证控制请求的生产判断逻辑，不覆盖 HTTP 传输、JSON 序列化或 MP4 编码。
