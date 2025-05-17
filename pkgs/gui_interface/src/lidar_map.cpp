#include "lidar_map.hpp"
#include <chrono>
#include <cmath>
#include <stdexcept>

#define LIDAR_TOPIC "/cardinal_perception/filtered_scan"

GLuint compileShader(GLenum type, const char *src)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        throw std::runtime_error(std::string("Shader compile error: ") + log);
    }
    return shader;
}

const char *vertex_src = R"(
#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 uView;
uniform mat4 uProj;

void main() {
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

const char *fragment_src = R"(
#version 330 core
out vec4 FragColor;
uniform vec4 uColor;
void main() {
    FragColor = uColor;
}
)";

LidarMap::LidarMap(rclcpp::Node &parent) : parent(parent) {}

void LidarMap::init()
{

    // OpenGL texture and framebuffer setup
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glGenTextures(1, &gl_tex);
    glBindTexture(GL_TEXTURE_2D, gl_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, map_wd, map_ht, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gl_tex, 0);

    glGenRenderbuffers(1, &gl_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, gl_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, map_wd, map_ht);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, gl_rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("Failed to set up GL framebuffer");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Shader setup
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertex_src);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragment_src);
    shader_program = glCreateProgram();
    glAttachShader(shader_program, vs);
    glAttachShader(shader_program, fs);
    glLinkProgram(shader_program);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint linked;
    glGetProgramiv(shader_program, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        char log[512];
        glGetProgramInfoLog(shader_program, 512, nullptr, log);
        throw std::runtime_error(std::string("Shader link error: ") + log);
    }

    // Create grid lines (static geometry)
    createGridLines();

    // point cloud VBO setup
    glGenVertexArrays(1, &points_vao);
    glBindVertexArray(points_vao);

    glGenBuffers(1, &points_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, points_vbo);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW); // Initially empty

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // PointCloud subscriber
    pcl_sub = parent.create_subscription<sensor_msgs::msg::PointCloud2>(
        LIDAR_TOPIC, rclcpp::SensorDataQoS{},
        [this](const sensor_msgs::msg::PointCloud2 &msg)
        {
            this->pclCallback(msg);
        });

    RCLCPP_INFO(parent.get_logger(), "LidarMap initialized (OpenGL mode)");
}

void LidarMap::createGridLines()
{
    constexpr float grid_step = 1.0f;
    constexpr float grid_min = -10.f * grid_step;
    constexpr float grid_max = 10.f * grid_step;

    // XY plane lines (parallel to X, sweeping along Y)
    for (float y = grid_min; y <= grid_max; y += grid_step)
    {
        grid_lines.push_back(grid_min);
        grid_lines.push_back(y);
        grid_lines.push_back(0.0f);
        grid_lines.push_back(grid_max);
        grid_lines.push_back(y);
        grid_lines.push_back(0.0f);
    }
    // XY plane lines (parallel to Y, sweeping along X)
    for (float x = grid_min; x <= grid_max; x += grid_step)
    {
        grid_lines.push_back(x);
        grid_lines.push_back(grid_min);
        grid_lines.push_back(0.0f);
        grid_lines.push_back(x);
        grid_lines.push_back(grid_max);
        grid_lines.push_back(0.0f);
    }

    // Z-axis line (0, 0, -z) to (0, 0, +z)
    grid_lines.push_back(0.0f);
    grid_lines.push_back(0.0f);
    grid_lines.push_back(-10.0f);
    grid_lines.push_back(0.0f);
    grid_lines.push_back(0.0f);
    grid_lines.push_back(10.0f);

    glGenVertexArrays(1, &grid_vao);
    glBindVertexArray(grid_vao);

    glGenBuffers(1, &grid_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, grid_vbo);
    glBufferData(GL_ARRAY_BUFFER, grid_lines.size() * sizeof(float), grid_lines.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);

    // Unbind to be safe
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

LidarMap::~LidarMap()
{
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &gl_tex);
    glDeleteRenderbuffers(1, &gl_rbo);
    glDeleteProgram(shader_program);

    if (points_vbo)
        glDeleteBuffers(1, &points_vbo);
    if (points_vao)
        glDeleteVertexArrays(1, &points_vao);

    if (grid_vbo)
        glDeleteBuffers(1, &grid_vbo);
    if (grid_vao)
        glDeleteVertexArrays(1, &grid_vao);
}

GLuint LidarMap::getTexture()
{
    renderToFramebuffer();
    return gl_tex;
}

void LidarMap::pclCallback(const sensor_msgs::msg::PointCloud2 &msg)
{
    static auto last_update = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update).count() < 1000 / 10) // only update at 10 Hz
        return;

    points_xyz.clear();
    const uint8_t *ptr = msg.data.data();
    size_t point_step = msg.point_step;

    for (size_t i = 0; i < msg.width * msg.height; ++i, ptr += point_step)
    {
        float x = *reinterpret_cast<const float *>(ptr + msg.fields[0].offset);
        float y = *reinterpret_cast<const float *>(ptr + msg.fields[1].offset);
        float z = *reinterpret_cast<const float *>(ptr + msg.fields[2].offset);
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
        {
            points_xyz.push_back(x);
            points_xyz.push_back(y);
            points_xyz.push_back(z);
        }
    }

    points_dirty = true;
    last_update = now;
}

void LidarMap::handleKeyState(const Uint8 *keys)
{
    const float yaw_speed = 0.02f;
    const float pitch_speed = 0.02f;
    const float zoom_speed = 0.5f;

    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT])
        yaw -= yaw_speed;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT])
        yaw += yaw_speed;
    if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP])
        pitch -= pitch_speed;
    if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN])
        pitch += pitch_speed;
    if (keys[SDL_SCANCODE_Q])
        radius += zoom_speed;
    if (keys[SDL_SCANCODE_E])
        radius -= zoom_speed;

    // Clamp limits
    pitch = std::clamp(pitch, -1.5f, 1.5f);
    radius = std::clamp(radius, 2.0f, 100.0f);
}

void LidarMap::renderToFramebuffer()
{
    // Camera settings
    float camX = radius * std::cos(pitch) * std::sin(yaw);
    float camY = radius * std::cos(pitch) * std::cos(yaw);
    float camZ = radius * std::sin(pitch);

    // Manually construct view matrix (could use glm in the future)
    float eye[] = {camX, camY, camZ};
    float center[] = {0.0f, 0.0f, 0.0f};
    float up[] = {0.0f, 0.0f, 1.0f};

    float f[3] = {
        center[0] - eye[0],
        center[1] - eye[1],
        center[2] - eye[2]};
    float fnorm = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    for (int i = 0; i < 3; ++i)
        f[i] /= fnorm;

    float s[3] = {
        f[1] * up[2] - f[2] * up[1],
        f[2] * up[0] - f[0] * up[2],
        f[0] * up[1] - f[1] * up[0]};
    float snorm = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
    for (int i = 0; i < 3; ++i)
        s[i] /= snorm;

    float u[3] = {
        s[1] * f[2] - s[2] * f[1],
        s[2] * f[0] - s[0] * f[2],
        s[0] * f[1] - s[1] * f[0]};

    float view[16] = {
        s[0], u[0], -f[0], 0.0f,
        s[1], u[1], -f[1], 0.0f,
        s[2], u[2], -f[2], 0.0f,
        -(s[0] * eye[0] + s[1] * eye[1] + s[2] * eye[2]),
        -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]),
        (f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2]),
        1.0f};

    // Perspective projection matrix
    float aspect = static_cast<float>(map_wd) / static_cast<float>(map_ht);
    float fov = 45.0f * (3.1415926f / 180.0f);
    float near = 0.1f, far = 100.0f;
    float tanHalfFovy = std::tan(fov / 2.0f);

    float proj[16] = {
        1.0f / (aspect * tanHalfFovy), 0, 0, 0,
        0, 1.0f / tanHalfFovy, 0, 0,
        0, 0, -(far + near) / (far - near), -1.0f,
        0, 0, -(2.0f * far * near) / (far - near), 0};

    // Setup render
    GLint old_fbo = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &old_fbo);
    GLint old_viewport[4];
    glGetIntegerv(GL_VIEWPORT, old_viewport);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, map_wd, map_ht);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shader_program);

    // Get uniform locations
    GLint loc_view = glGetUniformLocation(shader_program, "uView");
    GLint loc_proj = glGetUniformLocation(shader_program, "uProj");
    GLint loc_color = glGetUniformLocation(shader_program, "uColor");

    // Pass matrices
    glUniformMatrix4fv(loc_view, 1, GL_FALSE, view);
    glUniformMatrix4fv(loc_proj, 1, GL_FALSE, proj);

    // Draw point cloud
    if (points_dirty)
    {
        glBindBuffer(GL_ARRAY_BUFFER, points_vbo);
        glBufferData(GL_ARRAY_BUFFER, points_xyz.size() * sizeof(float), points_xyz.data(), GL_DYNAMIC_DRAW);
        points_dirty = false;
    }

    glUniform4f(loc_color, 0.0f, 1.0f, 0.0f, 1.0f); // Set point color to green

    glBindVertexArray(points_vao);
    glUniform4f(loc_color, 0.0f, 1.0f, 0.0f, 1.0f);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(2.5f);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points_xyz.size() / 3));
    glBindVertexArray(0);

    // Draw grid lines
    glUniform4f(loc_color, 1.0f, 1.0f, 1.0f, 0.3f); // Set grid color to white

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    glBindVertexArray(grid_vao);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(grid_lines.size() / 3));

    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glBindVertexArray(0);

    // Cleanup
    glDisableVertexAttribArray(0);
    glUseProgram(0);
    glDisable(GL_DEPTH_TEST);

    glBindFramebuffer(GL_FRAMEBUFFER, old_fbo);
    glViewport(old_viewport[0], old_viewport[1], old_viewport[2], old_viewport[3]);
}
