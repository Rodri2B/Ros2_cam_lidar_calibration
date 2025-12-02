void object_grabber(GLFWwindow* window, float windowWidth, float windowHeight,const glm::mat4 &projview, const std::vector<Object&> &obj ){

    ImGuiIO& io = ImGui::GetIO();
    
    double mouseX, mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);

    float x_ndc = (2.0f * mouseX) / windowWidth - 1.0f;
    float y_ndc = 1.0f - (2.0f * mouseY) / windowHeight;

    glm::mat4 inv_projview = glm::inverse(projview);
    glm::vec4 points[2];
    points[0] =  inv_projview*glm::vec4(x_ndc,y_ndc,-1.0,1.0);
    points[1] =  inv_projview*glm::vec4(x_ndc,y_ndc,1.0,1.0);


    //io.MousePos = ImVec2((float)mouseX, (float)mouseY); // Restore ImGui's mouse position

}

void