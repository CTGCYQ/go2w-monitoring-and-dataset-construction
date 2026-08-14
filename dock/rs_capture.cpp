// Go2-W 深度相机采集程序（独立于 ROS 节点）
//
// 用 librealsense C++ API 直接采集 Intel RealSense D435I 的彩色(RGB)和深度帧，
// 编码为 JPEG 保存到指定输出目录，供 80 服务器读取展示。
// 绕过有 bug 的 realsense2_camera ROS 节点。
//
// 用法: ./rs_capture <output_dir> [width] [height] [fps]
//
// 输出:
//   <output_dir>/color/frame_%06d.jpg   (RGB 彩色图)
//   <output_dir>/depth/frame_%06d.jpg   (深度图, 伪彩色编码, 近=红 远=蓝)
//   <output_dir>/latest_color.jpg       (最新帧, 覆盖写, 供 API 读取)
//   <output_dir>/latest_depth.jpg

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

// 把深度帧(16bit mm)转伪彩色 JPEG
// 深度热力图：近=红(暖)，远=蓝(冷)。平滑红→黄→绿→蓝过渡
static void heatmap(float t, int& r, int& g, int& b) {
    // t: 0=红, 0.33=黄, 0.66=绿, 1=蓝（线性插值）
    static const float HR[4] = {1.0f, 1.0f, 0.0f, 0.0f};
    static const float HG[4] = {0.0f, 1.0f, 1.0f, 0.0f};
    static const float HB[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    float x = t * 3.0f;
    int i = (int)x;
    if (i < 0) i = 0;
    if (i > 2) i = 2;
    float f = x - i;
    r = (int)((HR[i] + (HR[i+1]-HR[i])*f) * 255.0f);
    g = (int)((HG[i] + (HG[i+1]-HG[i])*f) * 255.0f);
    b = (int)((HB[i] + (HB[i+1]-HB[i])*f) * 255.0f);
    if (r > 255) r = 255; if (g > 255) g = 255; if (b > 255) b = 255;
    if (r < 0) r = 0; if (g < 0) g = 0; if (b < 0) b = 0;
}

static bool save_depth_jpeg(const rs2::depth_frame& depth, const std::string& path) {
    const int w = depth.get_width();
    const int h = depth.get_height();
    cv::Mat img(h, w, CV_8UC3);
    uint16_t* dp = (uint16_t*)depth.get_data();
    // 用官方 get_units() 获取深度单位（D435I 通常 0.001 = 毫米），避免硬编码错误
    const float depth_unit = depth.get_units();
    static bool unit_logged = false;
    if (!unit_logged) { unit_logged = true; printf("[rs_capture] depth_unit=%.6f m/raw\n", depth_unit); }
    // D435I 标准有效范围：0.3m ~ 10m。热力图：近=红(暖)，远=蓝(冷)
    const float min_d = 0.3f, max_d = 10.0f;
    const float range = max_d - min_d;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float d = dp[y * w + x] * depth_unit;  // 原始值 * 单位 = 米
            cv::Vec3b& px = img.at<cv::Vec3b>(y, x);
            if (d <= 0.001f) {
                px = cv::Vec3b(0, 0, 0);            // 无效深度 -> 黑色
                continue;
            }
            float t = (d - min_d) / range;
            if (t < 0.0f) t = 0.0f;                 // 过近 -> 最暖(红)
            if (t > 1.0f) t = 1.0f;                 // 过远 -> 最冷(蓝)
            int r, g, b;
            heatmap(t, r, g, b);
            px = cv::Vec3b((uchar)b, (uchar)g, (uchar)r);  // cv::Vec3b 是 BGR
        }
    }
    try {
        cv::imwrite(path, img);
        return true;
    } catch (...) {
        return false;
    }
}

int main(int argc, char** argv) {
    std::string out_dir = argc > 1 ? argv[1] : "/tmp/rs_out";
    int width = argc > 2 ? std::atoi(argv[2]) : 640;
    int height = argc > 3 ? std::atoi(argv[3]) : 480;
    int fps = argc > 4 ? std::atoi(argv[4]) : 15;

    std::string color_dir = out_dir + "/color";
    std::string depth_dir = out_dir + "/depth";
    system(("mkdir -p " + color_dir + " " + depth_dir).c_str());

    try {
        rs2::context ctx;
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
        cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
        pipe.start(cfg);

        std::cout << "[rs_capture] started: " << width << "x" << height
                  << "@" << fps << "fps -> " << out_dir << std::endl;
        std::cout << "[rs_capture] depth-color: NEAR=red WARM / FAR=blue COLD (v2.9 get_units)" << std::endl;

        long frame = 0;
        while (true) {
            rs2::frameset frames;
            if (!pipe.poll_for_frames(&frames)) {
                // 没有新帧就等一下
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                continue;
            }
            auto color = frames.get_color_frame();
            auto depth = frames.get_depth_frame();
            if (!color || !depth) continue;
            frame++;

            // 彩色帧 -> JPEG
            cv::Mat bgr(cv::Size(width, height), CV_8UC3, (void*)color.get_data());
            char cf[512], lc[512];
            std::snprintf(cf, sizeof(cf), "%s/frame_%06ld.jpg", color_dir.c_str(), frame);
            std::snprintf(lc, sizeof(lc), "%s/latest_color.jpg", out_dir.c_str());
            cv::imwrite(cf, bgr);
            std::ofstream(lc).close();
            std::rename(cf, lc);  // 覆盖最新帧

            // 深度帧 -> 伪彩色 JPEG
            char df[512], ld[512];
            std::snprintf(df, sizeof(df), "%s/frame_%06ld.jpg", depth_dir.c_str(), frame);
            std::snprintf(ld, sizeof(ld), "%s/latest_depth.jpg", out_dir.c_str());
            save_depth_jpeg(depth, df);
            std::ofstream(ld).close();
            std::rename(df, ld);

            if (frame % fps == 0) {
                std::cout << "[rs_capture] frame " << frame << std::endl;
            }
            // 限制保存数量，防止磁盘膨胀（保留最近 ~500 帧历史 + 2 个 latest）
            if (frame % (fps * 60) == 0) {
                std::string clean = "ls " + color_dir + " | head -n -600 | xargs -r -I{} rm -f " +
                                    color_dir + "/{} ; ls " + depth_dir +
                                    " | head -n -600 | xargs -r -I{} rm -f " + depth_dir + "/{}";
                system(clean.c_str());
            }
        }
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function()
                  << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
