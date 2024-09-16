#include <exception>
#include <iostream>
#include <boost/program_options.hpp>
#include <type_traits>
#include <chrono>
#include <thread>
#include <mutex>

#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_trigger_out.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_monitoring.h>
#include <metavision/hal/facilities/i_event_rate_activity_filter_module.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/sdk/cv/algorithms/activity_noise_filter_algorithm.h>
#include <metavision/sdk/cv/algorithms/trail_filter_algorithm.h>
#include <metavision/sdk/cv/algorithms/spatio_temporal_contrast_algorithm.h>
#include <metavision/hal/facilities/i_geometry.h>
#include <metavision/hal/facilities/i_events_stream_decoder.h>
#include <metavision/hal/facilities/i_event_decoder.h>
#include <metavision/hal/device/device.h>
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/events/event_ext_trigger.h>
#include <metavision/sdk/base/utils/error_utils.h>
#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>

#include <pcl/common/angles.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "event2d_id.hpp"
#include "eventSphere.hpp"

using namespace std::chrono_literals;

const size_t BUFFER_EVENT_SIZE = 10000;

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

auto dateTime = currentDateTime();

std::string logfilename = "/home/maxime/openeb-ws/dev/dataset_raw/SFERA/tracker"+ dateTime +".dat";

    // Open the log file name
std::ofstream flog(logfilename.c_str());

// Buffer commun aux deux caméras (taille arbitraire)
Metavision::Event2d_id* bufferEvent = new Metavision::Event2d_id[BUFFER_EVENT_SIZE];
int indexBuffer = 0; 

std::mutex mainMutex;

void quicksort(Metavision::Event2d_id* bufferEvent, int left, int right)
{
    if (left < right) {
        long long int pivot = bufferEvent[(left + right) / 2].t; // choisir un pivot
        int i = left - 1;
        int j = right + 1;
        while (true) {
            do {
                i++;
            } while (bufferEvent[i].t < pivot);
            do {
                j--;
            } while (bufferEvent[j].t > pivot);
            if (i >= j) {
                break;
            }
            std::swap(bufferEvent[i], bufferEvent[j]);
        }
        quicksort(bufferEvent, left, j);
        quicksort(bufferEvent, j + 1, right);
    }
}

void printAndEmptyBuffer()
{
    quicksort(bufferEvent, 0, indexBuffer);
    /*for(int i = 0; i < indexBuffer; i++)
    {
             std::cout << "Event received: coordinates (" << bufferEvent[i].x << ", " << bufferEvent[i].y << "), t: " << bufferEvent[i].t
       << ", polarity: " << bufferEvent[i].p << " Camera's id " << bufferEvent[i].idCam << std::endl;
    }*/
    // empty buffer and reset indexBuffer
    std::memset(bufferEvent, 0, sizeof(Metavision::Event2d_id) * BUFFER_EVENT_SIZE);
    //std::cout << "we emptied the buffer" << std::endl;
    indexBuffer = 0;
}


class EventAnalyzer {
public:
    std::mutex m;

    bool is_master;

    cv::Mat frame, frame_swap;

    // Display colors
    cv::Vec3b color_bg  = cv::Vec3b(52, 37, 30);
    cv::Vec3b color_on  = cv::Vec3b(236, 223, 216);
    cv::Vec3b color_off = cv::Vec3b(201, 126, 64);

    void setup_display(const int width, const int height) {
        frame      = cv::Mat(height, width, CV_8UC3);
        frame_swap = cv::Mat(height, width, CV_8UC3);
        frame.setTo(color_bg);
    }

    // Called from main Thread
    cv::Mat get_frame() {
        /* Swap images
        {
            std::unique_lock<std::mutex> lock(m);
            std::swap(frame, frame_swap);
            //frame.setTo(color_bg);
        }
        frame_swap.copyTo(display);*/
        return frame;
    }

    void set_frame(cv::Mat &display) {
            
        /*{ 
            std::unique_lock<std::mutex> lock(m);
            display.copyTo(frame); 
        }*/
       this->frame=display;
    }


    void setMasterMode(bool is_master){
        this->is_master = is_master;
    }

    void process_events(const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        // acquire lock

        {
            mainMutex.lock(); //Mutex commun aux deux threads process, pour remplir le buffer
            if(is_master)
            {
                for (auto it = begin; it != end; ++it) {
                //    std::cout << "we go there -- MASTER" << std::endl;
                    //std::cout << "indexBuffer = " << indexBuffer;
                    bufferEvent[indexBuffer] = Metavision::Event2d_id(it->x, it->y, it->p, it->t, 0);
                   // img.at<cv::Vec3b>(it->y, it->x) = (it->p) ? color_on : color_off;
                //    std::cout << bufferEvent[indexBuffer].to_string() << std::endl;
                    indexBuffer += 1;
                    if(indexBuffer > BUFFER_EVENT_SIZE)
                        printAndEmptyBuffer();
                 //   std::cout << " indexBuffer after = " << indexBuffer << std::endl;
                }
            } else {
                for (auto it = begin; it != end; ++it) {
                  //  std::cout << "we go there -- SLAVE" << std::endl;
                    bufferEvent[indexBuffer] = Metavision::Event2d_id(it->x, it->y, it->p, it->t, 1);
                    //bufferEvent[indexBuffer].to_string();
                   // img.at<cv::Vec3b>(it->y, it->x) = (it->p) ? color_on : color_off;
                  //  std::cout << bufferEvent[indexBuffer].to_string() << std::endl;
                    indexBuffer += 1;
                    if(indexBuffer > BUFFER_EVENT_SIZE)
                        printAndEmptyBuffer();
                    //std::cout << " indexBuffer after = " << indexBuffer << std::endl;
                }
            }
    
            mainMutex.unlock();
        }
    }
};

class CloudManager{
public:
    std::mutex m;

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;

    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr get_cloud(){
        std::unique_lock<std::mutex> lock(m);
        return this->cloud;
    }

    void set_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in){
        std::unique_lock<std::mutex> lock(m);
        this->cloud=cloud_in;
    }

};

namespace po = boost::program_options;
bool live_camera = true;
bool raw_record = true;
int main(int argc, char *argv[]) {

    if(argc < 2)
    {
        std::cerr << "Missing informations" << std::endl;
        std::cerr << "Using : " << argv[0] << " {SerialPortMaster (or .raw master)} {SerialPortSlave (or .raw slave)}" << std::endl;
        return -1;
    }

    // Open the slave camera first
    std::cout << "Opening camera slave..." << std::endl;
    std::unique_ptr<Metavision::Device> deviceSlaveCamera;
    try {if(live_camera==true){
        deviceSlaveCamera = Metavision::DeviceDiscovery::open(argv[2]);
    }
    else{deviceSlaveCamera = Metavision::DeviceDiscovery::open_raw_file(argv[2]);}
        
    } catch (Metavision::BaseException &e) {
        std::cerr << "Error exception: " << e.what() << std::endl;
    }
    if (!deviceSlaveCamera) {
        std::cerr << "Camera slave opening failed." << std::endl;
        return 1;
    }
    std::cout << "Camera slave open." << std::endl;

    if(live_camera==true){
    //Set slave mode
    Metavision::I_CameraSynchronization *i_camera_synchronization_slave =
        deviceSlaveCamera->get_facility<Metavision::I_CameraSynchronization>();

    if (i_camera_synchronization_slave->set_mode_slave()) {
        std::cout << "Set mode Slave successful." << std::endl;
    } else {
        std::cerr << "Could not set Slave mode. Master/slave might not be supported by your camera" << std::endl;
        return 3;
    }
    }

    auto out_raw_file_path_slave = "/home/maxime/openeb-ws/dev/dataset_raw/SFERA/RAW/slave"+ dateTime +".raw";
    // Initialize slave event stream
    Metavision::I_EventsStream *i_eventsstream_slave = deviceSlaveCamera->get_facility<Metavision::I_EventsStream>();
    if (i_eventsstream_slave) { //add recording node here
        if (out_raw_file_path_slave != "" && raw_record) {
            i_eventsstream_slave->log_raw_data(out_raw_file_path_slave);
        }
    } else {
        std::cerr << "Could not initialize events stream of camera slave." << std::endl;
        return 3;
    }
    
    i_eventsstream_slave->start();

    // Open the master camera
    std::cout << "Opening camera master..." << std::endl;
    std::unique_ptr<Metavision::Device> deviceMasterCamera;
    try {
        if(live_camera==true){
        deviceMasterCamera = Metavision::DeviceDiscovery::open(argv[1]);
    }
    else{deviceMasterCamera = Metavision::DeviceDiscovery::open_raw_file(argv[1]);}  
    } catch (Metavision::BaseException &e) {
        std::cerr << "Error exception: " << e.what() << std::endl;
    }
    if (!deviceMasterCamera) {
        std::cerr << "Camera master opening failed." << std::endl;
        return 1;
    }
    std::cout << "Camera master open." << std::endl;

    if(live_camera==true){
    // Set master mode
    Metavision::I_CameraSynchronization *i_camera_synchronization_master =
        deviceMasterCamera->get_facility<Metavision::I_CameraSynchronization>();

     if (i_camera_synchronization_master->set_mode_master()) {
        std::cout << "Set mode Master successful." << std::endl;
    } else {
        std::cerr << "Could not set Master mode. Master/slave might not be supported by your camera" << std::endl;
        return 3;
    }
    }

    auto out_raw_file_path_master = "/home/maxime/openeb-ws/dev/dataset_raw/SFERA/RAW/master +" + dateTime +".raw";
    // Initialize master event stream
    Metavision::I_EventsStream *i_eventsstream_master = deviceMasterCamera->get_facility<Metavision::I_EventsStream>();
    if (i_eventsstream_master) { //add recording node here
        if (out_raw_file_path_master != "" && raw_record) {
           i_eventsstream_master->log_raw_data(out_raw_file_path_master);
        }
    } else {
        std::cerr << "Could not initialize events stream of camera slave." << std::endl;
        return 3;
    }

    i_eventsstream_master->start();

    // get camera slave geometry
    Metavision::I_Geometry *i_geometry_slave = deviceSlaveCamera->get_facility<Metavision::I_Geometry>();
    if (!i_geometry_slave) {
        std::cerr << "Could not retrieve camera slave geometry." << std::endl;
        return 4;
    }

    // get camera master geometry
    Metavision::I_Geometry *i_geometry_master = deviceMasterCamera->get_facility<Metavision::I_Geometry>();
    if (!i_geometry_master) {
        std::cerr << "Could not retrieve camera master geometry." << std::endl;
        return 4;
    }

    //accumulation time & frame rate
    double fps              = 50;
    const std::uint32_t acc = 20000;// static_cast<std::uint32_t>(std::round(1.f / fps * 1000000)); 
    const std::uint32_t threshold_noise = 5000;
    const std::uint32_t threshold_trail = 500000;
    const std::uint32_t threshold_stc = 10000;

    // Instantiate framer object for slave
    EventAnalyzer event_analyzer_slave;
    event_analyzer_slave.setMasterMode(false);
    event_analyzer_slave.setup_display(i_geometry_slave->get_width(), i_geometry_slave->get_height());
    auto frame_gen_slave = Metavision::PeriodicFrameGenerationAlgorithm(i_geometry_slave->get_width(), i_geometry_slave->get_height(), acc, fps);
    //auto noise_filter_slave = Metavision::ActivityNoiseFilterAlgorithm<>(i_geometry_slave->get_width(), i_geometry_slave->get_height(), threshold_noise);
    //auto trail_filter_slave = Metavision::TrailFilterAlgorithm(i_geometry_slave->get_width(), i_geometry_slave->get_height(), threshold_trail);
    auto stc_filter_slave = Metavision::SpatioTemporalContrastAlgorithm(i_geometry_slave->get_width(), i_geometry_slave->get_height(), threshold_stc);
    // Get the handler of CD events from slave camera
    Metavision::I_EventDecoder<Metavision::EventCD> *i_cddecoder_slave =
        deviceSlaveCamera->get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();

    if (i_cddecoder_slave) {
        i_cddecoder_slave->add_event_buffer_callback(
            [&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
                std::vector<Metavision::EventCD> output;//,output2;
                //noise_filter_slave.process_events(begin, end, std::back_inserter(output1));
                //trail_filter_slave.process_events(output1.begin(),output1.end(),std::back_inserter(output2));
                //stc_filter_slave.process_events(begin,end,std::back_inserter(output));
                frame_gen_slave.process_events(begin, end);//(begin, end);
                output.clear();
            });
    }

    std::cout << "Camera slave started." << std::endl;


    Metavision::I_Decoder *i_decoder_slave = deviceSlaveCamera->get_facility<Metavision::I_EventsStreamDecoder>();
    bool stop_decoding_slave               = false;
    bool stop_application_slave            = false;
    std::thread decoding_loop_slave([&]() {
        while (!stop_decoding_slave) {
            short ret = i_eventsstream_slave->poll_buffer();

            // Here we polled data, so we can launch decoding
            auto raw_data = i_eventsstream_slave->get_latest_raw_data();

            // This will trigger callbacks set on decoders: in our case EventAnalyzer.process_events
            if (raw_data) {
                i_decoder_slave->decode(raw_data->data(), raw_data->data() + raw_data->size());
            }
        }
    });

     // Instantiate framer object for slave
    EventAnalyzer event_analyzer_master;
    event_analyzer_master.setMasterMode(true);
    event_analyzer_master.setup_display(i_geometry_master->get_width(), i_geometry_master->get_height());
    auto frame_gen_master = Metavision::PeriodicFrameGenerationAlgorithm(i_geometry_master->get_width(), i_geometry_master->get_height(), acc, fps);
    //auto noise_filter_master = Metavision::ActivityNoiseFilterAlgorithm<>(i_geometry_master->get_width(), i_geometry_master->get_height(), threshold_noise);
    //auto trail_filter_master = Metavision::TrailFilterAlgorithm(i_geometry_master->get_width(), i_geometry_master->get_height(), threshold_trail);
    auto stc_filter_master = Metavision::SpatioTemporalContrastAlgorithm(i_geometry_master->get_width(), i_geometry_master->get_height(), threshold_stc);
    // Get the handler of CD events from slave camera
    Metavision::I_EventDecoder<Metavision::EventCD> *i_cddecoder_master =
        deviceMasterCamera->get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();

    if (i_cddecoder_master) {
        i_cddecoder_master->add_event_buffer_callback(
            [&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
                std::vector<Metavision::EventCD> output;//,output2;
                //noise_filter_master.process_events(begin, end, std::back_inserter(output1));
                //trail_filter_master.process_events(output1.begin(),output1.end(),std::back_inserter(output2));
                //stc_filter_master.process_events(begin,end,std::back_inserter(output));
                frame_gen_master.process_events(begin, end);//(begin, end);//
                output.clear();
            });    
    }

    std::cout << "Camera master started." << std::endl;

    Metavision::I_Decoder *i_decoder_master = deviceMasterCamera->get_facility<Metavision::I_EventsStreamDecoder>();
    bool stop_decoding_master               = false;
    bool stop_application_master            = false;
    std::thread decoding_loop_master([&]() {
        while (!stop_decoding_master) {
            short ret = i_eventsstream_master->poll_buffer();

            // Here we polled data, so we can launch decoding
            auto raw_data = i_eventsstream_master->get_latest_raw_data();

            // This will trigger callbacks set on decoders: in our case EventAnalyzer.process_events
            if (raw_data) {
                i_decoder_master->decode(raw_data->data(), raw_data->data() + raw_data->size());
            }
        }
    });

    cv::Mat K1(cv::Size(3,3),CV_64F,0.);
    K1.at<double>(0,0)=970.012;K1.at<double>(0,1)=0;K1.at<double>(0,2)=645.375;
    K1.at<double>(1,0)=0;K1.at<double>(1,1)=970.565;K1.at<double>(1,2)=365.778;
    K1.at<double>(2,0)=0;K1.at<double>(2,1)=0;K1.at<double>(2,2)=1;
    double xi1 =1.682;

    cv::Mat K2(cv::Size(3,3),CV_64F,0.);
    K2.at<double>(0,0)=972.311;K2.at<double>(0,1)=0;K2.at<double>(0,2)=618.132;
    K2.at<double>(1,0)=0;K2.at<double>(1,1)=969.822;K2.at<double>(1,2)=361.180;
    K2.at<double>(2,0)=0;K2.at<double>(2,1)=0;K2.at<double>(2,2)=1;
    double xi2 = 1.685;
    
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Global unit sphere"));
    viewer->initCameraParameters (); 
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.2);
    viewer->setCameraPosition(-0.149139, -1.39564, 2.06217,  0.0850799, -0.81015, -0.580016);
    viewer->setCameraFieldOfView(0.8575);
    viewer->setCameraClipDistances(0.00589725 , 5.89725); 

    Metavision::EventSphere sphere(K1,xi1,K2,xi2);
    sphere.generateTable(i_geometry_slave->get_width(), i_geometry_slave->get_height());

    int nFrameSlave = 0, nFrameMaster = 0, nCloud = 0;

    Metavision::Window window_slave("Slave Cam", i_geometry_slave->get_width(), i_geometry_slave->get_height(), Metavision::BaseWindow::RenderMode::BGR);
    window_slave.set_keyboard_callback(
        [&window_slave](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods) {
            if (action == Metavision::UIAction::RELEASE &&
                (key == Metavision::UIKeyEvent::KEY_ESCAPE || key == Metavision::UIKeyEvent::KEY_Q)) {
                window_slave.set_close_flag();
            }
        });
    frame_gen_slave.set_output_callback([&](Metavision::timestamp, cv::Mat &frame) { window_slave.show(frame); event_analyzer_slave.set_frame(frame); nFrameSlave++;});
 

    Metavision::Window window_master("Master Cam", i_geometry_slave->get_width(), i_geometry_slave->get_height(), Metavision::BaseWindow::RenderMode::BGR);
    window_master.set_keyboard_callback(
        [&window_master](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods) {
            if (action == Metavision::UIAction::RELEASE &&
                (key == Metavision::UIKeyEvent::KEY_ESCAPE || key == Metavision::UIKeyEvent::KEY_Q)) {
                window_master.set_close_flag();
            }
        });
    frame_gen_master.set_output_callback([&](Metavision::timestamp, cv::Mat &frame) { window_master.show(frame); event_analyzer_master.set_frame(frame); nFrameMaster++;});

    CloudManager cloud_m;
    //Thread de traitement de la sphère unitaire 
    std::thread sphere_processing([&]() {
        cv::Mat frame_slave, frame_master;      
        while (!stop_decoding_slave && !stop_application_slave) {
         if(nFrameMaster > 0 && nFrameSlave > 0) {
            frame_slave = event_analyzer_slave.get_frame();
            frame_master = event_analyzer_master.get_frame();
            cloud_m.cloud = sphere.SphereFromAccMapFast(frame_master,frame_slave);
            nCloud++;
         }
        }
    }); 

    //Thread de visualisation
    std::thread visualization([&]() {
        bool cloud_initialized = false;     
        while (!stop_decoding_slave && !stop_application_slave) {
         if(nCloud>0) {
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_sphere;
            cloud_sphere = cloud_m.cloud;
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_sphere);
            if(!cloud_initialized){
                viewer->addPointCloud<pcl::PointXYZRGB> (cloud_sphere,rgb,"Unit sphere");
                cloud_initialized=true;
            }
            else{
                viewer->updatePointCloud<pcl::PointXYZRGB>(cloud_sphere,rgb,"Unit sphere");
            }
            viewer->spinOnce(20);
            std::this_thread::sleep_for(20ms);
         }
        }
    }); 
   
        
    while(!stop_application_master && !stop_application_slave){

        static constexpr std::int64_t kSleepPeriodMs = 20;
        Metavision::EventLoop::poll_and_dispatch(kSleepPeriodMs);

    }
    // Wait end of decoding loop
    decoding_loop_master.join();

    // Wait end of decoding loop
    decoding_loop_slave.join();

    // Wait end of sphere loop
    sphere_processing.join();

    // Wait end of vizualization
    visualization.join();

    return 0;

}

