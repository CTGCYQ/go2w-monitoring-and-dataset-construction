// Loopback-only, request/reply depth transport. Latest frame stays in RAM.
#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <opencv2/imgcodecs.hpp>

class RawDepthStream {
    struct Frame { std::string fields; std::vector<char> pixels; std::vector<unsigned char> rgb_jpeg;
        std::chrono::steady_clock::time_point captured; };
    int listener=-1;
    std::atomic<bool> running{true};
    std::thread worker;
    std::mutex lock;
    std::shared_ptr<Frame> latest;
    static bool send_all(int fd,const void* data,size_t size) {
        const char* p=static_cast<const char*>(data);
        while(size) { auto n=send(fd,p,size,MSG_NOSIGNAL); if(n<=0)return false; p+=n;size-=n; }
        return true;
    }
    void serve() {
        std::vector<int> clients;
        while(running) {
            fd_set set; FD_ZERO(&set); FD_SET(listener,&set); int highest=listener;
            for(int fd:clients){FD_SET(fd,&set);if(fd>highest)highest=fd;}
            timeval wait{0,200000};
            if(select(highest+1,&set,nullptr,nullptr,&wait)<=0)continue;
            if(FD_ISSET(listener,&set)) {
                int fd=accept(listener,nullptr,nullptr);
                if(fd>=0) {
                    if(clients.size()>=4 || fd>=FD_SETSIZE)close(fd);
                    else {timeval timeout{0,500000};setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));clients.push_back(fd);}
                }
            }
            for(size_t i=0;i<clients.size();) {
                int client=clients[i];if(!FD_ISSET(client,&set)){++i;continue;}
                char request;bool ok=recv(client,&request,1,0)==1;
                std::shared_ptr<Frame> frame;
                {std::lock_guard<std::mutex> guard(lock);frame=latest;}
                if(ok && !frame){uint32_t zero=0;ok=send_all(client,&zero,4);}
                else if(ok) {
                    double age=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-frame->captured).count();
                    std::ostringstream h;h<<std::setprecision(16)<<"{\"source_age_ms\":"<<age<<',';
                    if(request=='R')h<<"\"rgbd_version\":1,\"rgb_format\":\"jpeg\",\"rgb_jpeg_bytes\":"<<frame->rgb_jpeg.size()<<',';
                    h<<frame->fields<<'}';auto header=h.str();uint32_t length=htonl(static_cast<uint32_t>(header.size()));
                    ok=send_all(client,&length,4)&&send_all(client,header.data(),header.size())&&send_all(client,frame->pixels.data(),frame->pixels.size());
                    if(ok && request=='R')ok=send_all(client,frame->rgb_jpeg.data(),frame->rgb_jpeg.size());
                }
                if(!ok){close(client);clients.erase(clients.begin()+i);}else ++i;
            }
        }
        for(int fd:clients)close(fd);
    }

public:
    explicit RawDepthStream(int port=8766) {
        listener=socket(AF_INET,SOCK_STREAM,0);
        if(listener<0)throw std::runtime_error("depth socket failed");
        int yes=1;setsockopt(listener,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0||listen(listener,2)<0) {
            close(listener);throw std::runtime_error("depth loopback bind failed");
        }
        worker=std::thread([this]{serve();});
    }
    ~RawDepthStream() { running=false;if(worker.joinable())worker.join();close(listener); }
    void publish(const rs2::depth_frame& depth,const rs2::video_frame& color,
                 std::chrono::steady_clock::time_point captured,double captured_unix) {
        auto frame=std::make_shared<Frame>();frame->captured=captured;
        int w=depth.get_width(),h=depth.get_height();
        const auto* bytes=static_cast<const char*>(depth.get_data());
        frame->pixels.resize(w*h*2);
        for(int y=0;y<h;++y)std::memcpy(frame->pixels.data()+y*w*2,bytes+y*depth.get_stride_in_bytes(),w*2);
        cv::Mat bgr(color.get_height(),color.get_width(),CV_8UC3,const_cast<void*>(color.get_data()),color.get_stride_in_bytes());
        if(!cv::imencode(".jpg",bgr,frame->rgb_jpeg,{cv::IMWRITE_JPEG_QUALITY,92}))return;
        auto in=depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        std::ostringstream s;s<<std::setprecision(16)
          <<"\"version\":1,\"format\":\"Z16_LE\",\"alignment\":\"color\",\"frame_id\":\"realsense_color_optical_frame\""
          <<",\"width\":"<<w<<",\"height\":"<<h<<",\"payload_bytes\":"<<w*h*2
          <<",\"meters_per_unit\":"<<depth.get_units()<<",\"frame_number\":"<<depth.get_frame_number()
          <<",\"captured_host_time_s\":"<<captured_unix<<",\"device_timestamp_ms\":"<<depth.get_timestamp()
          <<",\"timestamp_domain\":"<<static_cast<int>(depth.get_frame_timestamp_domain())
          <<",\"color_timestamp_ms\":"<<color.get_timestamp()
          <<",\"color_frame_number\":"<<color.get_frame_number()
          <<",\"fx\":"<<in.fx<<",\"fy\":"<<in.fy<<",\"ppx\":"<<in.ppx<<",\"ppy\":"<<in.ppy
          <<",\"distortion_model\":"<<static_cast<int>(in.model)<<",\"coefficients\":[";
        for(int i=0;i<5;++i){if(i)s<<',';s<<in.coeffs[i];}s<<']';
        frame->fields=s.str();std::lock_guard<std::mutex> guard(lock);latest=frame;
    }
};
