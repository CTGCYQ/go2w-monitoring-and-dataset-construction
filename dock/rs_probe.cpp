// Read-only RealSense factory calibration and raw Z16 validation.
// Prints aggregate JSON only; no images, raw depth, or trajectories are saved.
#include <librealsense2/rs.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

static std::string intrinsics_json(const rs2_intrinsics& in) {
    std::ostringstream s;
    s << std::setprecision(9) << "{\"width\":" << in.width << ",\"height\":" << in.height
      << ",\"fx\":" << in.fx << ",\"fy\":" << in.fy
      << ",\"ppx\":" << in.ppx << ",\"ppy\":" << in.ppy
      << ",\"distortion_model\":" << static_cast<int>(in.model) << ",\"coefficients\":[";
    for (int i=0;i<5;++i) { if(i) s << ','; s << in.coeffs[i]; }
    s << "]}";
    return s.str();
}

static std::string number_or_null(double x) {
    if (!std::isfinite(x)) return "null";
    std::ostringstream s; s << std::setprecision(9) << x; return s.str();
}

int main(int argc, char** argv) {
    int requested = argc>1 ? std::atoi(argv[1]) : 30;
    if (requested<5 || requested>300) { std::cerr << "frames must be 5..300\n"; return 2; }
    try {
        rs2::pipeline pipe;
        rs2::config config;
        config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,15);
        config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,15);
        auto profile=pipe.start(config);
        auto native_depth=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
        auto native_color=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto di=native_depth.get_intrinsics(), ci=native_color.get_intrinsics();
        auto extrinsic=native_depth.get_extrinsics_to(native_color);
        rs2::align align(RS2_STREAM_COLOR);
        for (int i=0;i<15;++i) pipe.wait_for_frames(5000);
        std::uint64_t valid=0, total=0, range_valid=0;
        int measured=0, aligned_frames=0, missing_center=0;
        double scale=0, minimum=std::numeric_limits<double>::infinity(), maximum=0;
        double center_min=std::numeric_limits<double>::infinity(),center_max=0;
        double max_sdk_error=0,max_timestamp_difference=0;
        bool same_domain=true;
        rs2_intrinsics aligned_intrinsics{};
        auto start=std::chrono::steady_clock::now();
        for (int i=0;i<requested;++i) {
            auto frames=pipe.wait_for_frames(5000);
            auto native=frames.get_depth_frame();
            if (!native || !frames.get_color_frame()) continue;
            if (native.get_profile().format()!=RS2_FORMAT_Z16) throw std::runtime_error("not Z16");
            scale=native.get_units();
            if (!(scale>0 && std::isfinite(scale))) throw std::runtime_error("invalid depth scale");
            auto aligned=align.process(frames);
            auto depth=aligned.get_depth_frame();
            auto color=aligned.get_color_frame();
            aligned_intrinsics=depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
            if (depth.get_width()==color.get_width() && depth.get_height()==color.get_height()) ++aligned_frames;
            bool common=depth.get_frame_timestamp_domain()==color.get_frame_timestamp_domain();
            same_domain=same_domain && common;
            if(common) max_timestamp_difference=std::max(max_timestamp_difference,std::abs(depth.get_timestamp()-color.get_timestamp()));
            const auto* bytes=static_cast<const std::uint8_t*>(depth.get_data());
            for(int y=0;y<depth.get_height();++y) {
                const auto* row=reinterpret_cast<const std::uint16_t*>(bytes+y*depth.get_stride_in_bytes());
                for(int x=0;x<depth.get_width();++x) {
                    ++total;
                    if(!row[x]) continue;
                    double m=row[x]*scale;
                    ++valid; if(m>=0.3 && m<=10) ++range_valid;
                    minimum=std::min(minimum,m);maximum=std::max(maximum,m);
                }
            }
            int cx=depth.get_width()/2, cy=depth.get_height()/2;
            std::vector<double> center;
            for(int y=cy-2;y<=cy+2;++y) for(int x=cx-2;x<=cx+2;++x) {
                const auto* row=reinterpret_cast<const std::uint16_t*>(bytes+y*depth.get_stride_in_bytes());
                double m=row[x]*scale;
                max_sdk_error=std::max(max_sdk_error,std::abs(m-depth.get_distance(x,y)));
                if(m>0) center.push_back(m);
            }
            if(center.empty()) ++missing_center;
            else {
                std::sort(center.begin(),center.end());double median=center[center.size()/2];
                center_min=std::min(center_min,median);center_max=std::max(center_max,median);
            }
            ++measured;
        }
        double duration=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
        pipe.stop();
        bool aligned_intrinsics_match=aligned_intrinsics.width==ci.width && aligned_intrinsics.height==ci.height
          && std::abs(aligned_intrinsics.fx-ci.fx)<0.001 && std::abs(aligned_intrinsics.fy-ci.fy)<0.001
          && std::abs(aligned_intrinsics.ppx-ci.ppx)<0.001 && std::abs(aligned_intrinsics.ppy-ci.ppy)<0.001;
        bool passed=measured==requested && aligned_frames==requested && aligned_intrinsics_match && valid>0 && max_sdk_error<1e-5 && same_domain && max_timestamp_difference<=1000.0/15.0;
        std::cout << std::setprecision(9) << "{\n\"requested_frames\":" << requested
          << ",\n\"measured_frames\":" << measured << ",\n\"aligned_frames\":" << aligned_frames
          << ",\n\"elapsed_s\":" << duration << ",\n\"observed_fps\":" << (duration>0?measured/duration:0)
          << ",\n\"depth_format\":\"Z16\",\n\"meters_per_raw_unit\":" << scale
          << ",\n\"raw_zero_is_invalid\":true,\n\"raw_depth_saved\":false,\n\"images_saved\":false"
          << ",\n\"depth_intrinsics\":" << intrinsics_json(di)
          << ",\n\"color_intrinsics\":" << intrinsics_json(ci)
          << ",\n\"aligned_depth_intrinsics\":" << intrinsics_json(aligned_intrinsics)
          << ",\n\"aligned_intrinsics_match_color\":" << (aligned_intrinsics_match?"true":"false")
          << ",\n\"depth_to_color\":{\"rotation_row_major\":[";
        for(int row=0;row<3;++row) for(int col=0;col<3;++col) {
            if(row||col) std::cout << ','; std::cout << extrinsic.rotation[col*3+row];
        }
        std::cout << "],\"translation_m\":[";
        for(int i=0;i<3;++i) { if(i) std::cout << ',';std::cout << extrinsic.translation[i]; }
        std::cout << "]},\n\"valid_pixel_fraction\":" << (total?static_cast<double>(valid)/total:0)
          << ",\n\"pixels_in_0_3_to_10m_fraction\":" << (total?static_cast<double>(range_valid)/total:0)
          << ",\n\"min_nonzero_depth_m\":" << number_or_null(minimum)
          << ",\n\"max_nonzero_depth_m\":" << maximum
          << ",\n\"center_patch_median_range_m\":[" << number_or_null(center_min) << ',' << number_or_null(center_max) << ']'
          << ",\n\"center_patch_missing_frames\":" << missing_center
          << ",\n\"max_raw_to_sdk_difference_m\":" << max_sdk_error
          << ",\n\"same_timestamp_domain\":" << (same_domain?"true":"false")
          << ",\n\"max_depth_color_timestamp_difference_ms\":" << max_timestamp_difference
          << ",\n\"passed\":" << (passed?"true":"false") << "\n}\n";
        return passed?0:1;
    } catch(const std::exception& e) {
        std::cerr << "RealSense probe failed: " << e.what() << '\n';return 1;
    }
}
