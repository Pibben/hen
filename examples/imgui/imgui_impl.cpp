//
// Created by per on 2019-03-03.
//

#include <cstdio>
#include <iostream>
#include <veclib.h>
#include <vector>
#include <tuple>
#include <hen.h>
#include <cfw/cfw.h>
#include <utils.h>
#include <stdcomp/shaders.h>
#include "imgui/imgui.h"

static VecLib::Vector4f vecFromInt32(uint32_t in) {
    uint8_t a = (in >> 24) & 0xff;
    uint8_t b = (in >> 16) & 0xff;
    uint8_t g = (in >> 8) & 0xff;
    uint8_t r = in & 0xff;

    return {r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f};
}

class ImguiVertexShader {
private:
    enum class InTraits { POSITION_INDEX = 0, TEXTURE_INDEX = 1, COLOR_INDEX = 2 };

    uint16_t mWidth;
    uint16_t mHeight;

public:
    using InType = typename std::tuple<VecLib::Vector2f, VecLib::Vector2f, uint32_t>;
    using OutType = typename std::tuple<VecLib::Vector4f, VecLib::Vector2f, VecLib::Vector4f>;

    enum class Traits { POSITION_INDEX = 0, TEXTURE_INDEX = 1, COLOR_INDEX = 2 };

    ImguiVertexShader(uint16_t width, uint16_t height) : mWidth(width), mHeight(height) {}

    OutType operator()(const InType& in) const {
        VecLib::Vector2f pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const uint32_t& color = std::get<static_cast<int>(InTraits::COLOR_INDEX)>(in);

        pos[0] = 2.0f * pos[0] / mWidth - 1.0f;
        pos[1] = -(2.0f * pos[1] / mHeight - 1.0f);

        return std::make_tuple(VecLib::Vector4f(pos, 0.0f, 1.0f), tex, vecFromInt32(color));
    }
};

class ImguiFragmentShader {
private:
    enum class InTraits { POSITION_INDEX = 0, TEXTURE_INDEX = 1, COLOR_INDEX = 2};

    RGBATextureSampler<VecLib::Vector4f> mTextureSampler;

public:
    using InType = std::tuple<VecLib::Vector4f, VecLib::Vector2f, VecLib::Vector4f>;
    using OutType = std::tuple<VecLib::Vector4f>;

    enum class Traits { COLOR_INDEX = 0, DEPTH_INDEX = 1 };

    explicit ImguiFragmentShader(uint8_t* pixels, uint16_t width, uint16_t height) : mTextureSampler(pixels, width, height) {}

    OutType operator()(const InType& in) const {
        const VecLib::Vector4f& pos = std::get<static_cast<int>(InTraits::POSITION_INDEX)>(in);
        const VecLib::Vector2f& tex = std::get<static_cast<int>(InTraits::TEXTURE_INDEX)>(in);
        const VecLib::Vector4f& color = std::get<static_cast<int>(InTraits::COLOR_INDEX)>(in);

        auto tex_color = mTextureSampler.get(tex[0], tex[1]);

        return std::make_tuple(tex_color * color);
    }
};

int main() {
    const uint16_t width = 1280;
    const uint16_t height = 720;

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    io.KeyMap[ImGuiKey_Tab]         = static_cast<int>(cfw::Keys::TAB);
    io.KeyMap[ImGuiKey_LeftArrow]   = static_cast<int>(cfw::Keys::ARROWLEFT);
    io.KeyMap[ImGuiKey_RightArrow]  = static_cast<int>(cfw::Keys::ARROWRIGHT);
    io.KeyMap[ImGuiKey_UpArrow]     = static_cast<int>(cfw::Keys::ARROWUP);
    io.KeyMap[ImGuiKey_DownArrow]   = static_cast<int>(cfw::Keys::ARROWDOWN);
    io.KeyMap[ImGuiKey_PageUp]      = static_cast<int>(cfw::Keys::PAGEUP);
    io.KeyMap[ImGuiKey_PageDown]    = static_cast<int>(cfw::Keys::PAGEDOWN);
    io.KeyMap[ImGuiKey_Home]        = static_cast<int>(cfw::Keys::HOME);
    io.KeyMap[ImGuiKey_End]         = static_cast<int>(cfw::Keys::END);
    io.KeyMap[ImGuiKey_Insert]      = static_cast<int>(cfw::Keys::INSERT);
    io.KeyMap[ImGuiKey_Delete]      = static_cast<int>(cfw::Keys::_DELETE);
    io.KeyMap[ImGuiKey_Backspace]   = static_cast<int>(cfw::Keys::BACKSPACE);
    io.KeyMap[ImGuiKey_Space]       = static_cast<int>(cfw::Keys::SPACE);
    io.KeyMap[ImGuiKey_Enter]       = static_cast<int>(cfw::Keys::ENTER);
    io.KeyMap[ImGuiKey_Escape]      = static_cast<int>(cfw::Keys::ESC);
    io.KeyMap[ImGuiKey_A]           = static_cast<int>(cfw::Keys::A);
    io.KeyMap[ImGuiKey_C]           = static_cast<int>(cfw::Keys::C);
    io.KeyMap[ImGuiKey_V]           = static_cast<int>(cfw::Keys::V);
    io.KeyMap[ImGuiKey_X]           = static_cast<int>(cfw::Keys::X);
    io.KeyMap[ImGuiKey_Y]           = static_cast<int>(cfw::Keys::Y);
    io.KeyMap[ImGuiKey_Z]           = static_cast<int>(cfw::Keys::Z);

    io.DisplaySize = ImVec2(width, height);

    unsigned char* pixels;
    int texWidth, texHeight;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &texWidth, &texHeight);

    cfw::Window disp(width, height);
    PixelBuffer<uint8_t> framebuffer(width, height, 3);

    AlphaRasterShader rasterShader(&framebuffer);
    ImguiVertexShader vertexShader(width, height);
    ImguiFragmentShader fragmentShader(pixels, texWidth, texHeight);

    using Vertex = std::tuple<VecLib::Vector2f, VecLib::Vector2f, uint32_t>;

    bool going = true;

    disp.setMouseCallback([&io](uint32_t x, uint32_t y, uint32_t keys, int32_t wheel){
        io.MousePos = ImVec2((float)x, (float)y);
        io.MouseDown[0] = (keys & 1) != 0;
        io.MouseDown[1] = (keys & 2) != 0;
        io.MouseDown[2] = (keys & 4) != 0;
    });

    disp.setKeyCallback([&io](cfw::Keys key, bool pressed) {
        io.KeysDown[static_cast<int>(key)] = pressed;

        io.KeyCtrl  = (key == cfw::Keys::CTRLLEFT || key == cfw::Keys::CTRLRIGHT) && pressed;
        io.KeyAlt   = key == cfw::Keys::ALT && pressed;
        io.KeyShift = (key == cfw::Keys::SHIFTLEFT || key == cfw::Keys::SHIFTRIGHT) && pressed;
    });

    disp.setCharCallback([&io](const char* chars) {
        assert(chars != nullptr);
        io.AddInputCharactersUTF8(chars);
    });

    disp.setCloseCallback([&going]() { going = false; });

    Renderer renderer;
    renderer.setCulling(false);

    while(going) {

        ImGui::NewFrame();
        //ImGui::Begin("Hello, world!");
        //ImGui::Text("This is some useful text.");
        ImGui::ShowDemoWindow();
        //ImGui::End();

        ImGui::Render();

        auto *data = ImGui::GetDrawData();

        std::vector<Vertex> verts;

        for (int n = 0; n < data->CmdListsCount; n++) {
            const ImDrawList *cmd_list = data->CmdLists[n];
            const ImDrawVert *vtx_buffer = cmd_list->VtxBuffer.Data;
            const ImDrawIdx *idx_buffer = cmd_list->IdxBuffer.Data;

            for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++) {
                const ImDrawCmd *pcmd = &cmd_list->CmdBuffer[cmd_i];

                for (int c = 0; c < pcmd->ElemCount; c += 3) {
                    for (int v = 0; v < 3; ++v) {
                        const auto i = idx_buffer[c + v];
                        const auto &imv = vtx_buffer[i];

                        Vertex vertex;
                        auto &pos = std::get<0>(vertex);
                        auto &uv = std::get<1>(vertex);
                        auto &col = std::get<2>(vertex);

                        pos.x() = imv.pos.x;
                        pos.y() = imv.pos.y;


                        uv.x() = imv.uv.x;
                        uv.y() = imv.uv.y;

                        col = imv.col;

                        verts.push_back(vertex);
                    }
                }

                idx_buffer += pcmd->ElemCount;
            }
        }

        renderer.render(verts, vertexShader, fragmentShader, rasterShader);

        disp.render(framebuffer.data(), width, height);
        disp.paint();
    }
}
