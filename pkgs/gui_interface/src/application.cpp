#include "application.hpp"

Application::Application() : rclcpp::Node("control_gui"), status_toggle()
                             // Motor info Subscribers
                             ,
                             track_right_sub(this->create_subscription<TalonInfo>(
                                 "track_right_info", 10, [this](const TalonInfo &msg)
                                 { update_info(msg, 0); })),
                             track_left_sub(this->create_subscription<TalonInfo>(
                                 "track_left_info", 10, [this](const TalonInfo &msg)
                                 { update_info(msg, 1); })),
                             trencher_sub(this->create_subscription<TalonInfo>(
                                 "trencher_info", 10, [this](const TalonInfo &msg)
                                 { update_info(msg, 2); })),
                             hopper_belt_sub(this->create_subscription<TalonInfo>(
                                 "hopper_belt_info", 10, [this](const TalonInfo &msg)
                                 { update_info(msg, 3); })),
                             hopper_actuator_sub(this->create_subscription<TalonInfo>(
                                 "hopper_info", 10, [this](const TalonInfo &msg)
                                 { update_info(msg, 4); }))
                             // Robot Status Publisher
                             ,
                             robot_status_pub(this->create_publisher<std_msgs::msg::Int8>(
                                 "robot_status", 10))
                             // Live feed camera feed
                             ,
                             image_sub(this->create_subscription<sensor_msgs::msg::CompressedImage>(
                                 "camera/image_compressed", 10, [this](const sensor_msgs::msg::CompressedImage &msg)
                                 {
                                     if (msg.format != "jpg") {
                                         RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg.format.c_str());
                                     }
                                     this->imageCallback(msg); })),
                             camera_selection(this->create_publisher<std_msgs::msg::Int8>("camera_select", 10)), current_camera(4), last_camera(4)
                             // 3D Point Cloud Map
                             ,
                             pcl_map_sub(this->create_subscription<sensor_msgs::msg::PointCloud2>("lidarmap_chunk", 10, [this](const sensor_msgs::msg::PointCloud2 &msg)
                                                                                                  { this->pclCallback(msg); }))
{

    if (!wind)
    {
        throw std::runtime_error("Could not create SDL window");
    }
    if (!rend)
    {
        throw std::runtime_error("Could not create SDL Renderer");
    }
    if (!video_tex)
    {
        throw std::runtime_error("Could not create SDL Texture: video_tex");
    }
    if (!pcl_tex)
    {
        throw std::runtime_error("Could not create SDL Texture: pcl_tex");
    }

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    ImPlot::CreateContext();

    if (!ImGui_ImplSDL2_InitForSDLRenderer(wind, rend))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to Initialize ImGui backend");
        throw std::runtime_error("Failed to Initialize ImGui backend");
    }
    if (!ImGui_ImplSDLRenderer2_Init(rend))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to Initialize ImGui Renderer backend");
        throw std::runtime_error("Failed to Initialize ImGui Renderer backend");
    }

    // Initializing Timers for update gui and sending robot status
    update_timer = create_wall_timer(25ms, [this]()
                                     { this->update(); });
    status_update_timer = create_wall_timer(100ms, [this]()
                                            {
        std_msgs::msg::Int8 state;
        state.data = robot_status;
        robot_status_pub->publish(state); });

    init_elements();

    RCLCPP_DEBUG(this->get_logger(), "Initialized GUI");
}

Application::~Application()
{
    if (video_tex)
    {
        SDL_DestroyTexture(video_tex);
    }
    if (pcl_tex)
    {
        SDL_DestroyTexture(pcl_tex);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    if (rend)
    {
        SDL_DestroyRenderer(rend);
    }
    if (wind)
    {
        SDL_DestroyWindow(wind);
    }
    SDL_Quit();
}

void Application::init_elements()
{
    // Robot Status Toggle Switch
    status_toggle = MultiToggle(&robot_status, status_options, "status_switch", "Status");
    status_toggle.SetColors(toggle_cols);

    right_track = std::make_shared<MotorInfo>(BUFFER_SIZE);
    left_track = std::make_shared<MotorInfo>(BUFFER_SIZE);
    trencher = std::make_shared<MotorInfo>(BUFFER_SIZE);
    hopper_belt = std::make_shared<MotorInfo>(BUFFER_SIZE);
    hopper_actuator = std::make_shared<MotorInfo>(BUFFER_SIZE);

    // Info Plots
    plots = InfoPlot(right_track, left_track, trencher, hopper_belt, hopper_actuator, BUFFER_SIZE);
}

void Application::update()
{

    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
        ImGui_ImplSDL2_ProcessEvent(&e);

        if (e.type == SDL_QUIT)
        {
            throw std::runtime_error("Goodbye");
        }
    }

    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // Robot Status
    {
        ImGui::SetNextWindowPos(ImVec2(0, 700), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(300, 380), ImGuiCond_Always);
        ImGui::Begin("Robot Status Control", nullptr,
                     ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoCollapse);

        int last_state = robot_status;
        status_toggle.Render();

        if (last_state != robot_status)
        {
            auto msg = std_msgs::msg::Int8();
            msg.data = robot_status;
            robot_status_pub->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Robot status changed to: %d", robot_status);
        }

        ImGui::End();
    }

    // Motor Status Information
    {
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(300, 350), ImGuiCond_Always);
        ImGui::Begin("Motor Status Information", nullptr,
                     ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoMouseInputs |
                         ImGuiWindowFlags_NoCollapse);

        // Talon SRX - Phoenix 5 Motors
        {
            ImGui::Text("Talon SRX - Phoenix 5");

            ImGui::Text("Hopper Actuator:");
            if (!hopper_actuator->position.empty())
            {
                ImGui::Text("\tPosition: %.3f", hopper_actuator->position.back().y);
            }
            else
            {
                ImGui::Text("\tPosition: ");
            }

            ImGui::Text("\tAverage Temp: %.2f", hopper_actuator->ave_temp);
        }

        // Talon FX - Phoenix 6 Motors
        {
            ImGui::Text("Talon FX - Phoenix 6");

            ImGui::Text("Right Track:");
            ImGui::Text("\tAverage Temp: %.2f", right_track->ave_temp);

            ImGui::Text("Left Track:");
            ImGui::Text("\tAverage Temp: %.2f", left_track->ave_temp);

            ImGui::Text("Trencher:");
            ImGui::Text("\tAverage Temp: %.2f", trencher->ave_temp);

            ImGui::Text("Hopper Belt:");
            ImGui::Text("\tAverage Temp: %.2f", hopper_belt->ave_temp);
        }

        ImGui::End();
    }

    // Motor Data plots
    {
        plots.Render();
    }

    // Camera feed
    {
        ImGui::SetNextWindowPos(ImVec2(300, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(900, 525), ImGuiCond_Always);
        ImGui::Begin("Video Display", nullptr,
                     ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoFocusOnAppearing |
                         ImGuiWindowFlags_NoBringToFrontOnFocus);

        ImGui::Image((ImTextureID)video_tex, ImVec2(cam_width, cam_height));
        ImGui::End();

        // Menu to select which camera is being viewed (if viewing feed).
        // Will turn on and off publisher on robot to reduce traffic
        {
            ImGui::SetNextWindowPos(ImVec2(955, 27), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(240, 480), ImGuiCond_Always);
            ImGui::Begin("Camera Feed Selection", nullptr,
                         ImGuiWindowFlags_NoResize);

            for (int i = 0; i < (int)camera_options.size(); i++)
            {
                if (ImGui::RadioButton(camera_options[i].c_str(), current_camera == i))
                {
                    current_camera = i;
                    if (current_camera != last_camera)
                    {
                        last_camera = current_camera;

                        // publish camera selection
                        auto msg = std_msgs::msg::Int8();
                        msg.data = static_cast<int8_t>(current_camera);
                        camera_selection->publish(msg);

                        RCLCPP_INFO(this->get_logger(), "changing states to: %d", current_camera);
                    }
                }
            }

            ImGui::Text("Selected Camera: %s", camera_options[current_camera].c_str());
            ImGui::End();
        }
    }

    // Point Cloud Map
    {
        ImGui::SetNextWindowSize(ImVec2(900, 900), ImGuiCond_Always);
        ImGui::Begin("LiDAR Map", nullptr,
                     ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize
                     // ImGuiWindowFlags_NoFocusOnAppearing |
                     // ImGuiWindowFlags_NoBringToFrontOnFocus
        );

        ImGui::Image((ImTextureID)pcl_tex, ImVec2(cam_width, cam_height));
        ImGui::End();
    }

    ImGui::Render();
    SDL_SetRenderDrawColor(rend, 0, 0, 0, 255);
    SDL_RenderClear(rend);
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), rend);
    SDL_RenderPresent(rend);
}

void Application::update_info(const TalonInfo &msg, const int id)
{
    switch (id)
    {
    case 0:
        right_track->add_point(msg);
        break;
    case 1:
        left_track->add_point(msg);
        break;
    case 2:
        trencher->add_point(msg);
        break;
    case 3:
        hopper_belt->add_point(msg);
        break;
    case 4:
        hopper_actuator->add_point(msg);
        break;
    }
}

void Application::imageCallback(const sensor_msgs::msg::CompressedImage &msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);
    if (!video_tex)
    {

        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
            return;
        }

        if (!video_tex || frame.cols != cam_width || frame.rows != cam_height)
        {
            cam_width = frame.cols;
            cam_height = frame.rows;

            if (video_tex)
            {
                SDL_DestroyTexture(video_tex);
            }

            video_tex = SDL_CreateTexture(
                rend,
                SDL_PIXELFORMAT_BGR24,
                SDL_TEXTUREACCESS_STREAMING,
                cam_width, cam_height);

            if (!video_tex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create SDL Texture for live camera");
                return;
            }
        }
    }

    void *pixels;
    int pitch;
    if (SDL_LockTexture(video_tex, nullptr, &pixels, &pitch) == 0)
    {
        uint8_t *dst = static_cast<uint8_t *>(pixels);
        for (int y = 0; y < cam_height; ++y)
        {
            for (int x = 0; x < cam_width; ++x)
            {
                int idx = y * pitch + x * 3;
                dst[idx + 0] = frame.at<cv::Vec3b>(y, x)[2]; // R
                dst[idx + 1] = frame.at<cv::Vec3b>(y, x)[1]; // G
                dst[idx + 2] = frame.at<cv::Vec3b>(y, x)[0]; // B
            }
        }
        SDL_UnlockTexture(video_tex);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to lock SDL texture");
    }
}

void Application::pcl2ToPCL(const sensor_msgs::msg::PointCloud2 &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // clear cloud
    cloud->clear();

    // get the point cloud data from message
    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    tmp_cloud.width = msg.width;
    tmp_cloud.height = msg.height;
    tmp_cloud.is_dense = msg.is_dense;
    tmp_cloud.points.resize(msg.data.size() / msg.point_step);

    // iterate through message and extract points
    const uint8_t *ptr = &msg.data[0];
    for (size_t i = 0; i < tmp_cloud.points.size(); ++i, ptr += msg.point_step)
    {
        float x = *reinterpret_cast<const float *>(ptr + msg.fields[0].offset); // TODO
        float y = *reinterpret_cast<const float *>(ptr + msg.fields[1].offset); // TODO
        float z = *reinterpret_cast<const float *>(ptr + msg.fields[2].offset); // TODO

        tmp_cloud.points[i].x = x;
        tmp_cloud.points[i].y = y;
        tmp_cloud.points[i].z = z;
    }

    *cloud = tmp_cloud;
}

void Application::pclCallback(const sensor_msgs::msg::PointCloud2 &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl2ToPCL(msg, cloud);

    if (!pcl_tex)
    {
        cam_width = 800;
        cam_height = 600;

        pcl_tex = SDL_CreateTexture(
            rend,
            SDL_PIXELFORMAT_RGBA8888,
            SDL_TEXTUREACCESS_STREAMING,
            cam_width, cam_height);

        if (!pcl_tex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create SDL Texture for point cloud");
            return;
        }
    }

    // blank framebuffer
    std::vector<uint32_t> frame_buffer(cam_width * cam_height, 0x00000000); // black

    // project 3d in 2d space
    for (const auto &point : cloud->points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }

        // perspective projection
        float fov = 90.0f;
        float aspect = static_cast<float>(cam_width) / cam_height;
        float focal_len = 1.0f / tan((fov * 0.5f) * M_PI / 180.0f);

        // transform the point (basic ex: no rotation)
        float screen_x = (point.x / -point.z) * focal_len * cam_width / aspect + cam_width / 2;
        float screen_y = (point.y / -point.z) * focal_len * cam_height / aspect + cam_height / 2;

        // map to framebuffer (clip to screen dimensions)
        int pixel_x = static_cast<int>(screen_x);
        int pixel_y = static_cast<int>(screen_y);

        if (pixel_x >= 0 && pixel_x < cam_width && pixel_y >= 0 && pixel_y < cam_height)
        {
            frame_buffer[pixel_y * cam_width + pixel_x] = 0xFFFFFF00;
        }
    }

    // update SDL texture
    void *pixels;
    int pitch;
    if (SDL_LockTexture(pcl_tex, nullptr, &pixels, &pitch) == 0)
    {
        memcpy(pixels, frame_buffer.data(), cam_width * cam_height * sizeof(uint32_t));
        SDL_UnlockTexture(pcl_tex);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to lock SDL texture (pcl)");
    }
}