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
    uint8_t a = static_cast<uint8_t>(in >> 24);
    uint8_t b = static_cast<uint8_t>(in >> 16);
    uint8_t g = static_cast<uint8_t>(in >> 8);
    uint8_t r = static_cast<uint8_t>(in);

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
    enum class InTraits { /*POSITION_INDEX = 0,*/ TEXTURE_INDEX = 1, COLOR_INDEX = 2};

    RGBATextureSampler<VecLib::Vector4f> mTextureSampler;

public:
    using InType = std::tuple<VecLib::Vector4f, VecLib::Vector2f, VecLib::Vector4f>;
    using OutType = std::tuple<VecLib::Vector4f>;

    enum class Traits { COLOR_INDEX = 0, DEPTH_INDEX = 1 };

    explicit ImguiFragmentShader(uint8_t* pixels, uint16_t width, uint16_t height) : mTextureSampler(pixels, width, height) {}

    OutType operator()(const InType& in) const {
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
    ImguiFragmentShader fragmentShader(pixels, static_cast<uint16_t>(texWidth), static_cast<uint16_t>(texHeight));

    using Vertex = std::tuple<VecLib::Vector2f, VecLib::Vector2f, uint32_t>;

    bool going = true;

    disp.setMouseCallback([&io](uint32_t x, uint32_t y, uint32_t keys, int32_t wheel){
        io.MousePos = ImVec2((float)x, (float)y);
        io.MouseDown[0] = (keys & 1) != 0;
        io.MouseDown[1] = (keys & 2) != 0;
        io.MouseDown[2] = (keys & 4) != 0;
        io.MouseWheel = static_cast<float>(wheel) / 10.0f;
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
        std::vector<uint32_t> indices;

        for (int n = 0; n < data->CmdListsCount; n++) {
            const ImDrawList *cmd_list = data->CmdLists[n];
            const ImDrawVert *vtx_buffer = cmd_list->VtxBuffer.Data;

            for (int v = 0; v < cmd_list->VtxBuffer.size(); ++v) {
                const auto &imv = vtx_buffer[v];

                verts.push_back({{imv.pos.x, imv.pos.y},
                                 {imv.uv.x, imv.uv.y},
                                 imv.col});
            }

            const ImDrawIdx *idx_buffer = cmd_list->IdxBuffer.Data;

            for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++) {
                const ImDrawCmd *pcmd = &cmd_list->CmdBuffer[cmd_i];

                for (unsigned int c = 0; c < pcmd->ElemCount; c++) {
                    indices.push_back(idx_buffer[c]);
                }

                idx_buffer += pcmd->ElemCount;
            }
        }

        framebuffer.fill(0);

        renderer.renderIndexed(verts, indices, vertexShader, fragmentShader, rasterShader);

        disp.render(framebuffer.data(), width, height);
        disp.paint();
    }
}
