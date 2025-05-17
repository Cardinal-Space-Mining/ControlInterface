#include "application.hpp"
#include <bit>
#include <glad/glad.h>

Application::Application() : rclcpp::Node("control_gui")
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
                                 { update_info(msg, 4); })),
                             // Hopper capacity
                             hopper_capacity_sub(this->create_subscription<std_msgs::msg::Float32>(
                                 "hopper_cap_info", 10, [this](const std_msgs::msg::Float32 &msg)
                                 { update_hopper_cap(msg); }))
                             // Robot Status Publisher
                             ,
                             robot_status_pub(this->create_publisher<std_msgs::msg::Int8>(
                                 "robot_status", 10)),
                             motor_config_pub(this->create_publisher<std_msgs::msg::Int8>(
                                 "config_motors", 10))
                             // Live feed camera feed
                             ,
                             image_sub(this->create_subscription<sensor_msgs::msg::CompressedImage>(
                                 "camera/image_compressed", 10, [this](const sensor_msgs::msg::CompressedImage &msg)
                                 {
                                     if (msg.format != "jpg") {
                                         RCLCPP_ERROR(this->get_logger(), "Unsupported encoding: %s", msg.format.c_str());
                                     }
                                     this->imageCallback(msg); })),
                             camera_selection(this->create_publisher<std_msgs::msg::Int8>("camera_select", 10)), current_camera(4), last_camera(4),
                             map(*this)
{
    wind = SDL_CreateWindow("Mission Control", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!wind)
        throw std::runtime_error("Failed to create SDL window");

    gl_context = SDL_GL_CreateContext(wind);
    if (!gl_context)
        throw std::runtime_error("Failed to create OpenGL context");

    SDL_GL_MakeCurrent(wind, gl_context);

    if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress))
        throw std::runtime_error("Failed to initialize GLAD");

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGui_ImplSDL2_InitForOpenGL(wind, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    #if GUI_PUBLISH_HEARTBEAT
    heartbeat = this->create_publisher<std_msgs::msg::Int32>("heartbeat", 10);
    #endif

    update_timer = create_wall_timer(25ms, [this]()
                                     { update(); });
    status_update_timer = create_wall_timer(100ms, [this]()
                                            {
        std_msgs::msg::Int8 msg;
        msg.data = robot_status;
        robot_status_pub->publish(msg);

        #if GUI_PUBLISH_HEARTBEAT
        heartbeat->publish(std_msgs::msg::Int32{}.set__data(250));
        #endif
    });

    init_elements();
}

Application::~Application()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    if (video_tex)
        glDeleteTextures(1, &video_tex);
    if (gl_context)
        SDL_GL_DeleteContext(gl_context);
    if (wind)
        SDL_DestroyWindow(wind);

    SDL_Quit();
}

void Application::init_elements()
{
    map.init();

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

    // Capacity
    cap = hopper_capacity(hopper_capacity_sub);
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

    if (lidar_map_hovered)
    {
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        map.handleKeyState(keys);
    }

    ImGui_ImplOpenGL3_NewFrame();
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

        if (ImGui::Button("Config Motors"))
        {
            auto cmsg = std_msgs::msg::Int8();
            cmsg.data = 0;
            motor_config_pub->publish(cmsg);

            RCLCPP_INFO(this->get_logger(), "Published msg to config motor");
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

        // Battery status
        {
            double battery_cap = (right_track->bus_volt.end()->y + left_track->bus_volt.end()->y + trencher->bus_volt.end()->y + hopper_belt->bus_volt.end()->y) / 4;
            if (battery_cap > 16.0)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 255, 0, 255));
            }
            else if (battery_cap > 15.0)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 0, 255, 255));
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(255, 0, 0, 255));
            }

            ImGui::Text("Battery Status: %.2f V", battery_cap);

            ImGui::PopStyleColor();
        }

        ImGui::End();
    }

    // Hopper Fullness Detection
    {
        ImGui::SetNextWindowPos(ImVec2(0, 350), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(300, 350), ImGuiCond_Always);
        ImGui::Begin("Hopper Fullness & Estimate Moved", nullptr,
                     ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize);
        ImGui::Text("Current Capacity: %.3f%%", cap.get_capacity());

        ImGui::End();
    }

    // Motor Data plots
    {
        plots.Render();
    }

    // Point Cloud Map
    {
        ImGui::SetNextWindowPos(ImVec2(300, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(map.getWd(), map.getHt()), ImGuiCond_Always);
        ImGui::Begin("LiDAR Map", nullptr,
                     ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoResize |
                     // ImGuiWindowFlags_NoFocusOnAppearing |
                        ImGuiWindowFlags_NoBringToFrontOnFocus
        );

        GLuint tex = map.getTexture();
        if (!tex)
        {
            RCLCPP_WARN(this->get_logger(), "map.getPCL is null");
        }
        else
        {
            ImGui::Image((ImTextureID)tex, ImVec2(map.getWd(), map.getHt()));
        }
        lidar_map_hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByPopup);
        ImGui::End();
    }

    // Camera feed
    {
        ImGui::SetNextWindowPos(ImVec2(1200, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(640, 480), ImGuiCond_Always);
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
            ImGui::SetNextWindowPos(ImVec2(1200, 480), ImGuiCond_Always);
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

    ImGui::Render();
    glViewport(0, 0, WIDTH, HEIGHT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(wind);
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
    default:
        RCLCPP_DEBUG(this->get_logger(), "switch statement in %s hit default case!", __func__);
        assert(false);
    }
}

void Application::update_hopper_cap(const std_msgs::msg::Float32 &msg)
{
    cap.set_capacity(msg.data);
}

void Application::imageCallback(const sensor_msgs::msg::CompressedImage &msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_COLOR);
    if (frame.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
        return;
    }

    cam_width = frame.cols;
    cam_height = frame.rows;

    if (video_tex == 0)
    {
        glGenTextures(1, &video_tex);
        glBindTexture(GL_TEXTURE_2D, video_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cam_width, cam_height, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
    }
    else
    {
        glBindTexture(GL_TEXTURE_2D, video_tex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, cam_width, cam_height, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
}
