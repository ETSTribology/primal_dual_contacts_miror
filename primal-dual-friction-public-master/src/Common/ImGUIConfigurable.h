#pragma once
#include <imgui/imgui.h>

namespace ImGui {
inline bool SliderFloatType(const char* label,
                            float* v,
                            float v_min,
                            float v_max,
                            const char* format = "%.3f",
                            ImGuiSliderFlags flags = 0) {
	return ImGui::SliderScalar(label, ImGuiDataType_Float, v, &v_min, &v_max,
	                           format, flags);
}
inline bool SliderFloatType(const char* label,
                            double* v,
                            double v_min,
                            double v_max,
                            const char* format = "%.3f",
                            ImGuiSliderFlags flags = 0) {
	return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max,
	                           format, flags);
}
inline bool InputFloatType(const char* label,
                           float* v,
                           float step = 0.0f,
                           float step_fast = 0.0f,
                           const char* format = "%.3f",
                           ImGuiInputTextFlags flags = 0) {
	flags |= ImGuiInputTextFlags_CharsScientific;
	return InputScalar(label, ImGuiDataType_Float, (void*)v,
	                   (void*)(step > 0.0f ? &step : NULL),
	                   (void*)(step_fast > 0.0f ? &step_fast : NULL), format,
	                   flags);
}
inline bool InputFloatType(const char* label,
                           double* v,
                           double step = 0.0,
                           double step_fast = 0.0,
                           const char* format = "%.3f",
                           ImGuiInputTextFlags flags = 0) {
	flags |= ImGuiInputTextFlags_CharsScientific;
	return InputScalar(label, ImGuiDataType_Double, (void*)v,
	                   (void*)(step > 0.0f ? &step : NULL),
	                   (void*)(step_fast > 0.0f ? &step_fast : NULL), format,
	                   flags);
}
inline bool SliderFloatType3(const char* label,
                             float* v,
                             float v_min,
                             float v_max,
                             const char* format = "%.3f",
                             ImGuiSliderFlags flags = 0) {
	return ImGui::SliderScalarN(label, ImGuiDataType_Float, v, 3, &v_min,
	                            &v_max, format, flags);
}
inline bool SliderFloatType3(const char* label,
                             double* v,
                             double v_min,
                             double v_max,
                             const char* format = "%.3f",
                             ImGuiSliderFlags flags = 0) {
	return ImGui::SliderScalarN(label, ImGuiDataType_Double, v, 3, &v_min,
	                            &v_max, format, flags);
}
inline bool InputFloatType3(const char* label,
                            float v[3],
                            const char* format = "%.3f",
                            ImGuiInputTextFlags flags = 0) {
	return InputScalarN(label, ImGuiDataType_Float, v, 3, NULL, NULL, format,
	                    flags);
}
inline bool InputFloatType3(const char* label,
                            double v[3],
                            const char* format = "%.3f",
                            ImGuiInputTextFlags flags = 0) {
	return InputScalarN(label, ImGuiDataType_Double, v, 3, NULL, NULL, format,
	                    flags);
}
}  // namespace ImGui

namespace FrictionFrenzy {
class ImGUIConfigurable {
   public:
	virtual void showImGUIMenu() = 0;
};
}  // namespace FrictionFrenzy
