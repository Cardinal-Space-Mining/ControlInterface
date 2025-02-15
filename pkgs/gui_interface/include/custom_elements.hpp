#ifndef CUSTOM_ELEMENTS_HPP
#define CUSTOM_ELEMENTS_HPP

#include <vector>
#include <iostream>
#include <string>
#include <array>

#include "imgui.h"
#include "imgui_internal.h"

enum class BASE_COLORS {
    RED,
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
    PURPLE,
    BLACK,
    WHITE,
    NONE
};

/*
 * In cases where ImU32 cannot be used for colors,
 * this vector of ImVec4 colors will be used.
*/
constexpr std::array<ImVec4,8> COLORS = {
    ImVec4(1.00f, 0.20f, 0.20f, 1.00f),
    ImVec4(1.00f, 0.63f, 0.00f, 1.00f),
    ImVec4(1.00f, 1.00f, 0.00f, 1.00f),
    ImVec4(0.20f, 1.00f, 0.20f, 1.00f),
    ImVec4(0.20f, 0.20f, 1.00f, 1.00f),
    ImVec4(0.63f, 0.00f, 1.00f, 1.00f),
    ImVec4(0.00f, 0.00f, 0.00f, 1.00f),
    ImVec4(1.00f, 1.00f, 1.00f, 1.00f)
};

constexpr float ANIMATION_SPEED = 0.08F;

/*
 * Basic custom element class object. All actual instances are
 * separate classes extending this class. This class should not
 * be used as an element to be output in window.
 * 
 * Proper use of this class would be creating a vector of different
 * custom elements to be in 1 window. 
*/
class CustomElement {
    public:
        CustomElement() = default;

        virtual void Render() const = 0;

        virtual ~CustomElement() = default;
};

/*
 * Instantiated Custom Element class object for outputting text in
 * window. Note: This is not actually custom, some elements may be
 * unique, but most syntax and format is standard ImGui practice and
 * functions.
*/
class TextOutput : CustomElement {
    public:
        TextOutput(std::string text) : output(text), col(BASE_COLORS::NONE) {}
        TextOutput(std::string text, BASE_COLORS col) : output(text), col(col) {}

        void Render() const override {
            if (col == BASE_COLORS::NONE) {
                ImGui::Text("%s", output.c_str());
            } else {
                ImGui::TextColored(COLORS.at(static_cast<int>(col)), "%s", output.c_str());
            }
        }

        void SetColor(BASE_COLORS color) { col = color; }

    private:
        std::string output;
        BASE_COLORS col;
};

/*
 * Instantiated Custom Element class object for inputting text in
 * window. Note: This is not actually custom, some elements may be
 * unique, but most syntax and format is standard ImGui practice and
 * functions.
*/
class TextInput : CustomElement {
    public:
        TextInput(const char* id, char* buffer, int* buffersize)
            : id(id), buffer(buffer), buffersize(buffersize) {}

        void Render() const override {
            ImGui::InputText(id, buffer, *buffersize);
        }
        
    private:
        const char* id;
        char* buffer;
        int* buffersize;
};

/*
 * The ToggleButton class creates a Toggle Switch element with 2 options.
 * The options could be used as ON/OFF for example.
 * 
 * At this moment, there are only 2 color options: white or green. It would
 * not be difficult to add in the ability to use the enum class BASE_COLORS
 * or custom color options.
*/
class ToggleButton : CustomElement{
    public:
        ToggleButton(const char* str_id, const char* label, bool* v) : toggle(v), id(str_id), label(label) {}

        // Renders a toggle switch
        void Render() const override {
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImDrawList* draw_list = ImGui::GetWindowDrawList();

            float height = ImGui::GetFrameHeight();
            float width = height * 1.55f;
            float radius = height * 0.50f;

            if (ImGui::InvisibleButton(id, ImVec2(width, height))) {
                *toggle = !*toggle;
            }

            float t = *toggle ? 1.0f : 0.0f;

            ImGuiContext& g = *ImGui::GetCurrentContext();
            if (g.LastActiveId == g.CurrentWindow->GetID(id)) {
                float t_anim = ImSaturate(g.LastActiveIdTimer / ANIMATION_SPEED);
                t = *toggle ? (t_anim) : (1.0f - t_anim);
            }

            ImU32 col_bg;
            if (ImGui::IsItemHovered()) {
                col_bg = ImGui::GetColorU32(ImLerp(ImVec4(0.78f, 0.78f, 0.78f, 1.0f), ImVec4(0.64f, 0.83f, 0.34f, 1.0f), t));
            } else {
                col_bg = ImGui::GetColorU32(ImLerp(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), ImVec4(0.56f, 0.83f, 0.26f, 1.0f), t));
            }

            draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.5f);
            draw_list->AddCircleFilled(ImVec2(p.x + radius + t * (width - radius * 2.0f), p.y + radius), radius - 1.5f, IM_COL32(255, 255, 255, 255));

            ImGui::SameLine();

            ImGui::Text("%s is set to %s", label, (toggle ? "ON" : "OFF"));
        }

    private:
        bool* toggle;
        const char* id;
        const char* label;
};

/*
 * The MultiToggle class creates a Toggle Switch element with more than
 * 2 options: ON/OFF for example. Instead, this MultiToggle can be used
 * as: Autonomous, Disabled, Teleop for controlling a robot for example.
 * 
 * By default there are only 3 color options shown in a scale from White
 * to solid green. To use alternate colors, call SetColors() with multitoggle
 * class object using a vector of enum class BASE_COLORS. There currently
 * is not a implementation of custom colors.
 */
class MultiToggle : CustomElement {
    public:
        MultiToggle() {}

        MultiToggle(int* current_pos,std::vector<std::string> options, const char* id, const char* label) 
            : options(options), current_pos(current_pos), id(id), label(label) {}

        void Render() const override {
            ImVec2 p = ImGui::GetCursorScreenPos();
            ImDrawList* draw_list = ImGui::GetWindowDrawList();

            float height = ImGui::GetFrameHeight();
            float width = 2 * (height * 1.55f);
            float radius = height * 0.50f;
            float segment_width = width / options.size();

            ImGui::InvisibleButton(id, ImVec2(width, height));
            bool is_active = ImGui::IsItemActive();

            if (is_active) {
                float mouse_x = ImGui::GetIO().MousePos.x;
                float relative_x = mouse_x - p.x;

                int new_pos = static_cast<int>(relative_x / segment_width);
                new_pos = ImClamp(new_pos, 0, static_cast<int>(options.size()) - 1);
                *current_pos = new_pos;
            }

            float t = *current_pos / static_cast<float>(options.size() - 1);

            ImU32 col_bg;

            if (!colors.empty() && colors.size() == options.size()) {
                col_bg = ImGui::GetColorU32(COLORS.at(static_cast<int>(colors.at(*current_pos))));
            } else {
                col_bg = ImGui::IsItemHovered()
                ? ImGui::GetColorU32(ImLerp(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), ImVec4(0.3f, 0.8f, 0.3f, 1.0f), t))
                : ImGui::GetColorU32(ImLerp(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), ImVec4(0.6f, 0.7f, 0.3f, 1.0f), t));
            }

            draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.5f);
            draw_list->AddCircleFilled(ImVec2(p.x + radius + t * (width - radius * 2.0f), p.y + radius), radius - 1.5f, IM_COL32(255, 255, 255, 255));
            ImGui::SameLine();
            ImGui::Text("%s: %s", label, options.at(*current_pos).c_str());
        }

        void SetColors(std::vector<BASE_COLORS> cols) { colors = cols; }

    private:
        std::vector<std::string> options;
        std::vector<BASE_COLORS> colors;
        int* current_pos;
        const char* id;
        const char* label;
};

#endif