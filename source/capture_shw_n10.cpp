// Live SHW acquisition using the sensor's factory NVIDIA ISP tuning.
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <csignal>
#include <chrono>
#include "camera_transition_lock.h"
using namespace Argus;
namespace fs = std::filesystem;
static volatile std::sig_atomic_t stopping = 0;
static void stop_capture(int) { stopping = 1; }
static void ok(Status s, const char* what) {
    if (s != STATUS_OK) throw std::runtime_error(std::string(what)+": "+std::to_string(s));
}
template<class I, class T> I* api(T* p) {
    auto* i=interface_cast<I>(p);
    if (!i) throw std::runtime_error("Missing Argus interface");
    return i;
}
int main(int argc, char** argv) {
    CameraTransitionLock transition;
    try {
        if (argc != 5) throw std::runtime_error("Usage: capture_shw_n10 CAMERA_INDEX NEW_OUTPUT_DIR FRAMES SAVE_EVERY");
        unsigned camera=std::stoul(argv[1]), count=std::stoul(argv[3]), every=std::stoul(argv[4]);
        if ((camera!=2 && camera!=3) || !count || count>100000 || !every)
            throw std::runtime_error("Invalid camera/frame arguments");
        fs::path output(argv[2]);
        if (!fs::create_directory(output)) throw std::runtime_error("Output exists");
        std::signal(SIGTERM,stop_capture); std::signal(SIGINT,stop_capture);
        UniqueObj<CameraProvider> provider(CameraProvider::create());
        auto cp=api<ICameraProvider>(provider.get());
        std::vector<CameraDevice*> devices; ok(cp->getCameraDevices(&devices),"enumerate");
        if(camera>=devices.size()) throw std::runtime_error("Missing SHW camera");
        std::vector<SensorMode*> modes;
        ok(api<ICameraProperties>(devices[camera])->getAllSensorModes(&modes),"modes");
        if(modes.empty() || api<ISensorMode>(modes[0])->getResolution()!=Size2D<uint32_t>(2560,1984))
            throw std::runtime_error("Unexpected SHW mode");
        UniqueObj<CaptureSession> session(cp->createCaptureSession(devices[camera]));
        auto cs=api<ICaptureSession>(session.get());
        UniqueObj<OutputStreamSettings> settings(cs->createOutputStreamSettings(STREAM_TYPE_EGL));
        auto oes=api<IEGLOutputStreamSettings>(settings.get());
        ok(oes->setPixelFormat(PIXEL_FMT_YCbCr_420_888),"YUV");
        ok(oes->setResolution(Size2D<uint32_t>(2560,1984)),"resolution");
        ok(oes->setMetadataEnable(true),"metadata");
        UniqueObj<OutputStream> stream(cs->createOutputStream(settings.get()));
        UniqueObj<EGLStream::FrameConsumer> consumer(EGLStream::FrameConsumer::create(stream.get()));
        auto fc=api<EGLStream::IFrameConsumer>(consumer.get());
        UniqueObj<Request> request(cs->createRequest(CAPTURE_INTENT_VIDEO_RECORD));
        auto rq=api<IRequest>(request.get()); ok(rq->enableOutputStream(stream.get()),"stream");
        auto ss=api<ISourceSettings>(rq->getSourceSettings());
        ok(ss->setSensorMode(modes[0]),"mode");
        ok(ss->setFrameDurationRange(Range<uint64_t>(33333333)),"frame duration");
        ok(ss->setExposureTimeRange(Range<uint64_t>(7500000)),"exposure");
        ok(ss->setGainRange(Range<float>(1)),"analog gain");
        auto ac=api<IAutoControlSettings>(rq->getAutoControlSettings());
        ok(ac->setIspDigitalGainRange(Range<float>(1)),"digital gain");
        ok(ac->setAwbMode(AWB_MODE_AUTO),"AWB");
        std::ofstream frames(output/"frames.csv");
        frames<<"index,timestamp_ns,host_monotonic_ns,exposure_ns,analog_gain,awb_cct,saved\n";
        ok(cs->repeat(request.get()),"repeat");
        struct RelockOnExit {
            CameraTransitionLock& lock;
            ~RelockOnExit() { lock.lock(); }
        } relock{transition};
        unsigned acquired=0;
        for(unsigned n=0; n<count+90 && !stopping; ++n) {
            UniqueObj<EGLStream::Frame> frame(fc->acquireFrame(10000000000ULL));
            auto fi=api<EGLStream::IFrame>(frame.get());
            if(n<90) continue;
            if(n==90) transition.unlock();
            auto md=api<ICaptureMetadata>(api<EGLStream::IArgusCaptureMetadata>(frame.get())->getMetadata());
            auto host=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            const bool saved=acquired%every==0;
            if(saved) {
                auto jpeg=api<EGLStream::IImageJPEG>(fi->getImage());
                fs::path path=output/(std::to_string(acquired)+"-rgb.jpg");
                ok(jpeg->writeJPEG(path.c_str()),"JPEG");
                fs::copy_file(path,output/"latest-tmp.jpg",fs::copy_options::overwrite_existing);
                fs::rename(output/"latest-tmp.jpg",output/"latest.jpg");
            }
            frames<<acquired<<','<<md->getSensorTimestamp()<<','<<host<<','<<md->getSensorExposureTime()<<','<<md->getSensorAnalogGain()<<','<<md->getAwbCct()<<','<<saved<<'\n';
            frames.flush(); if(!frames) throw std::runtime_error("Metadata write failed");
            if(!acquired) { std::ofstream ready(output/"ready"); ready<<md->getSensorTimestamp()<<'\n'; }
            ++acquired;
        }
        transition.lock();
        cs->stopRepeat(); ok(cs->waitForIdle(10000000000ULL),"idle");
        std::ofstream complete(output/"complete.json");
        complete<<"{\"backend\":\"live-argus-isp\",\"camera_index\":"<<camera<<",\"frames\":"<<acquired<<"}\n";
        return complete ? 0 : 1;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
