#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <shader_s.hpp>
#include "camera.hpp"
#include <iostream>
#include "Cube.hpp"
#include "draw.h"
using namespace std;
using namespace glm;
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
void drawGrid();
void drawCoord();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void bufferUpdate(unsigned int cubeVBO);
void processInput(GLFWwindow *window);
unsigned int loadTexture(const char *path);
unsigned int loadCubemap(vector<std::string> faces);
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
const int cube_num = 27;
const int robot_num = 1;
const int mass_num = 64;
vector<Mass> masses;
vector<Spring> springs;
vector<int> imMasses;
// timing
float deltaTime = 0.0f;    // time between current frame and last frame
float lastFrame = 0.0f;
// settings
double a = LENGTH;
double b = 0.1;
double c = 0.1;

GLdouble cubeVertices[mass_num * 3 * robot_num]; // cube_count * 8 points * 3 (x, y, z)
GLdouble myCube_color[cube_num * 24 * robot_num]; // cube_count * 8 points * 3 bit
GLuint cubeIndices[cube_num * 36 * robot_num];   // cube_count * 6 faces * 2 triangles * 3 indices
GLdouble myEdge_color[cube_num * 24 * robot_num];      // cube_count * 8 points * 3 bit
GLuint myEdge_indices[cube_num * 24 * robot_num]; // cube_count * 12 * 2
GLdouble myShade_vertex[cube_num * 24 * robot_num];
GLdouble myShade_color[cube_num * 24 * robot_num];
GLuint myShadeindices[cube_num * 36 * robot_num];


float skyboxVertices[] = {
    // positions
    -1.0f,  1.0f, -1.0f,
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,

    -1.0f, -1.0f,  1.0f,
    -1.0f, -1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f,  1.0f,
    -1.0f, -1.0f,  1.0f,

     1.0f, -1.0f, -1.0f,
     1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f,  1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,

    -1.0f, -1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f, -1.0f,  1.0f,
    -1.0f, -1.0f,  1.0f,

    -1.0f,  1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f, -1.0f,

    -1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
     1.0f, -1.0f,  1.0f
};

float planeVertices[] = {
    // positions          // texture Coords (note we set these higher than 1 (together with GL_REPEAT as texture wrapping mode). this will cause the floor texture to repeat)
     5.0f, -0.5f,  5.0f,  2.0f, 0.0f,
    -5.0f, -0.5f,  5.0f,  0.0f, 0.0f,
    -5.0f, -0.5f, -5.0f,  0.0f, 2.0f,

     5.0f, -0.5f,  5.0f,  2.0f, 0.0f,
    -5.0f, -0.5f, -5.0f,  0.0f, 2.0f,
     5.0f, -0.5f, -5.0f,  2.0f, 2.0f
};

vector<vector<GLuint>> faceIndices{
    {0, 1, 2},
    {0, 2, 3},
    
    {4, 5, 6},
    {4, 6, 7},
    
    {1, 2, 5},
    {2, 5, 6},
    
    {0, 4, 3},
    {3, 4, 7},
    
    {2, 3, 6},
    {3, 6, 7},
    
    {1, 0, 5},
    {0, 4, 5}
};

vector<vector<GLuint>> edgeIndices{
    {0, 1},
    {1, 2},
    
    {2, 3},
    {3, 0},
    
    {4, 5},
    {5, 6},
    
    {6, 7},
    {4, 7},
    
    {0, 4},
    {3, 7},
    
    {1, 5},
    {2, 6}
};
vector<Mass> mass0 = generateMass(0.1, 0.1, 0, 0, 0.1);
vector<Spring> spring0 = generateSpring(5000);

int main()
{
    createRobot(0, 0, 0.1);
    someStuffToMakesuretheDrawingWroking();
    for (const auto& edge: myEdge_indices) {
        cout << "edge: " << edge << endl;
    }
    //updateVertices();
//    for (const auto& spring: springs) {
//        cout << spring.m1 <<' '<< spring.m2 << endl;
//    }
//    for (const auto& i: imMasses){
//        cout << "masses index: " << i << endl;
//    }
//    for (const auto& mass: masses){
//        cout << "massPosition\n" << "x: " << mass.p[0] << "y: " << mass.p[1] <<"z: " <<  mass.p[2] << endl;
//   }
//    for (const auto& i: cubeIndices) {
//        cout << "triangle indices: " << i << endl;
//    }
//    for (const auto& i: cubeVertices) {
//        cout << "vertices: " << i << endl;
//    }
//    cout << "vertices " << sizeof(cubeVertices) << endl;
//    cout << "num indices: " << sizeof(cubeIndices) << endl;
//    cout << "number masses: " << masses.size() << endl;
//    cout << "number springs: " << springs.size() << endl;
//    cout <<"mass index: " << imMasses.size() << endl;
    
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    glEnable(GL_DEPTH_TEST);
    // build and compile our shader zprogram
    // ------------------------------------
    Shader planeShader("cube_0/planeShader.vs", "cube_0/planeShader.fs");
    Shader skyboxShader("cube_0/skybox.vs", "cube_0/skybox.fs");
    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    unsigned int skyboxVAO, skyboxVBO;
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    unsigned int floorTexture = loadTexture("resources/textures/bricks2.jpg");
    vector<std::string> faces
    {
        "resources/textures/skybox/right.jpg",
        "resources/textures/skybox/left.jpg",
        "resources/textures/skybox/top.jpg",
        "resources/textures/skybox/bottom.jpg",
        "resources/textures/skybox/front.jpg",
        "resources/textures/skybox/back.jpg",
    };
    unsigned int cubemapTexture = loadCubemap(faces);
    skyboxShader.use();
    skyboxShader.setInt("skybox", 0);    // load and create a texture
    // -------------------------
    
    unsigned int planeVAO, planeVBO;
    glGenVertexArrays(1, &planeVAO);
    glGenBuffers(1, &planeVBO);
    glBindVertexArray(planeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), &planeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindVertexArray(0);
    
     //draw cube stuff
    Shader cubeShader("cube_0/standardShader.vs", "cube_0/standardShader.fs");
    unsigned int cubeVAO, cubeVBO, cubeEBO;
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);
    glGenBuffers(1, &cubeEBO);
    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
    glEnableVertexAttribArray(0);
    
    Shader edgeShader("cube_0/lineShader.vs", "cube_0/lineShader.fs");
    unsigned int edgeVAO, edgeVBO, edgeEBO;
    glGenVertexArrays(1, &edgeVAO);
    glGenBuffers(1, &edgeVBO);
    glGenBuffers(1, &edgeEBO);
    glBindVertexArray(edgeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, edgeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(myEdge_indices), myEdge_indices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
    glEnableVertexAttribArray(0);
    
//    Shader cubeShader("cube_0/standardShader.vs", "cube_0/standardShader.fs");
//    unsigned int pointVAO, pointVBO;
//    glGenVertexArrays(1, &pointVAO);
//    glGenBuffers(1, &pointVBO);
//    glBindVertexArray(pointVAO);
//    glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_DYNAMIC_DRAW);
//    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
//    glEnableVertexAttribArray(0);
//    glBindBuffer(GL_ARRAY_BUFFER, 0);
//    glBindVertexArray(0);
//    glEnableVertexAttribArray(1);
//    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0);
    
    while (!glfwWindowShouldClose(window))
    {
        // input
        // -----
        //cubeUpdate(mass0, spring0, 1);
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        
        // render
        // ------
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 0.0f);
        
        if (true){
        // draw line
            updateRobot();
            updateVertices();
            cout << "global Time Now: " << T <<  endl;
            glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            // std::cout << "Global Time now: " << T << endl;

            glDepthFunc(GL_LESS);
            planeShader.use();
            planeShader.setInt("texture1", 0);
            model = glm::mat4(1.0f);
            model = glm::translate(model, vec3(0.5, 1, 0.5));
            planeShader.setMat4("model",  model);
            planeShader.setMat4("view", view);
            planeShader.setMat4("projection", projection);
            glBindVertexArray(planeVAO);
            glBindTexture(GL_TEXTURE_2D, floorTexture);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            glBindVertexArray(0);
            
            model = glm::rotate(model, glm::radians(270.0f), glm::vec3(1, 0, 0));
            model = glm::translate(model, vec3(-0.5, -0.5, -0.5));
            
//            glDepthFunc(GL_ALWAYS);
//            cubeShader.use();
//            cubeShader.setMat4("MVP", projection * view * model);
//            glPointSize(15.0f);
//            glBindVertexArray(pointVAO);
//            glDrawArrays(GL_POINT, 0, 64);
            // draw cube
            glDepthFunc(GL_ALWAYS);
            cubeShader.use();
            cubeShader.setMat4("MVP", projection * view * model);
            glBindVertexArray(cubeVAO);
            glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_DYNAMIC_DRAW);
            glDrawElements(GL_TRIANGLES, 12 * 3 * cube_num, GL_UNSIGNED_INT, 0); // 12 triangle 3 points 27 cube
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
            
            glDepthFunc(GL_ALWAYS);
            glEnable(GL_LINE_SMOOTH);
            edgeShader.use();
            edgeShader.setMat4("MVP", projection * view * model);
            glBindVertexArray(edgeVAO);
            glBindBuffer(GL_ARRAY_BUFFER, edgeVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), &cubeVertices, GL_DYNAMIC_DRAW);
            glDrawElements(GL_LINES, 12 * 2 * cube_num, GL_UNSIGNED_INT, 0); // 12 triangle 3 points 27 cube
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
            
//            for (int i=0; i < 28; i++){
//                Line line(vec3(mass0[cubeIndices[i][0]].p), mass0[cubeIndices[i][1]].p);
//                line.setMVP(projection * view * model);
//                line.setColor(vec3(0, 0, 1));
//                line.draw();
//            }
            
//            for (int i=0; i < 8; i++){
//                Point point(vec3(mass0[i].p));
//                point.setMVP(projection * view * model);
//                point.setColor(vec3(0, 1, 0));
//                if (mass0[i].p[2] < 0.01){
//                    point.setColor(vec3(0.1, 0.9, 0.2));
//                }
//                point.draw();
//            }
//
//            for (int i = 0; i < 12; i++) {
//                DrawFace face(mass0[faceIndices[i][0]].p, mass0[faceIndices[i][1]].p, mass0[faceIndices[i][2]].p);
//                face.setMVP(projection * view * model);
//                face.setColor(vec3(0.9, 0.4, 0.9));
//                face.draw();
//            }
            //updateRobot();
//            cout << masses.size() << endl;
//            for (int i = 0; i < masses.size(); i++) {
//                cout << "whathappend: " << i << endl;
//                point_vertices.push_back(masses[i].p[0]);
//                point_vertices.push_back(masses[i].p[1]);
//                point_vertices.push_back(masses[i].p[2]);
//            }
//            cout << sizeof(point_vertices) << endl;
//            cout << sizeof(double) << endl;
//            Point point;
//            point.setMVP(projection * view * model);
//            point.setColor(vec3(0.4, 0.4, 0.9));
//            point.draw();
            
//            for (int i = 0; i < springs.size(); i++) {
//                Line line(vec3(masses[springs[i].m1].p), vec3(masses[springs[i].m2].p));
//                line.setMVP(projection * view * model);
//                line.setColor(vec3(0, 0, 1));
//                line.draw();
//            }
            glDepthFunc(GL_EQUAL);
            skyboxShader.use();
            view = glm::mat4(glm::mat3(camera.GetViewMatrix())); // remove translation from the view matrix
            skyboxShader.setMat4("view", view);
            skyboxShader.setMat4("projection", projection);
            glBindVertexArray(skyboxVAO);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            glBindVertexArray(0);
            glDepthFunc(GL_LESS); // set depth function back to default
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }
    
    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
unsigned int loadTexture(char const * path)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

// loads a cubemap texture from 6 individual texture faces
// order:
// +X (right)
// -X (left)
// +Y (top)
// -Y (bottom)
// +Z (front)
// -Z (back)
// -------------------------------------------------------
unsigned int loadCubemap(vector<std::string> faces)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrComponents;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrComponents, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);
        }
        else
        {
            std::cout << "Cubemap texture failed to load at path: " << faces[i] << std::endl;
            stbi_image_free(data);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
}


void createCube (double x, double y, double z) {
    int n = (int)masses.size();
    // int n2 = (int)springs.size();
    // first create 8 masses
    vector<Mass> tempMasses(8);
    tempMasses[0] = {MASS,{x+LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[1] = {MASS,{x+LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[2] = {MASS,{x-LENGTH/2,y-LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[3] = {MASS,{x-LENGTH/2,y+LENGTH/2,z},{0,0,0},{0,0,0}};
    tempMasses[4] = {MASS,{x+LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[5] = {MASS,{x+LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[6] = {MASS,{x-LENGTH/2,y-LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    tempMasses[7] = {MASS,{x-LENGTH/2,y+LENGTH/2,z+LENGTH},{0,0,0},{0,0,0}};
    
    // check the mass created if coinciding with the previous cube if is
    // make m = 0 coinciding pushback the mass overlap, imMasses pushback
    // record all the indices
    vector<int> coinciding;
    for (int j = 0; j < tempMasses.size(); j++) {
        for (int i = 0; i < masses.size(); i++) {
            if (theSame(tempMasses[j], masses[i])) {
                tempMasses[j].m = 0;
                coinciding.push_back(i);
                imMasses.push_back(i);
            }
        }
    }
    
    // pushback the not coinciding masses
    int count = 0;
    for (int i = 0; i < tempMasses.size(); i++) {
        if(tempMasses[i].m != 0) {
            masses.push_back(tempMasses[i]);
            imMasses.push_back(n+count);
            count++;
        }
    }
    
    // create springs, this is for the nonconinciding masses
    for (int i = n; i < masses.size() - 1; i++) {
        for (int j = i+1; j < masses.size(); j++) {
            Spring s;
            s.k = springConstant;
            s.m1 = j;
            s.m2 = i;
            springs.push_back(s);
        }
    }
    
    // create springs, this is for the coinciding masses
    for (int i = 0; i < coinciding.size(); i++) {
        for (int j = n; j < masses.size(); j++) {
            Spring s;
            s.k = springConstant;
            s.m1 = coinciding[i];
            s.m2 = j;
            springs.push_back(s);
        }
    }
    for (Spring& spring: springs){
        spring.L0 = distance(masses[spring.m1].p, masses[spring.m2].p);
    }
}


// Robot is a set of cube
void createRobot(double x, double y, double z){
    for (int i = 0; i < DIM; i++){
        for (int j = 0; j < DIM; j++){
            for (int k = 0; k < DIM; k++){
                createCube((double)(i)/10.0 + x, (double)(j)/10.0 + y, (double)(k)/10.0 + z);
            }
        }
    }
}

void updateRobot() {
//    for (int i = 0; i < springs.size(); i++) {
//        if (i % 8 == 0) {
//            springs[i].a = a;
//            springs[i].b = b;
//            springs[i].c = c;
//            springs[i].L0 = springs[i].a + springs[i].b*sin(T*omega + springs[i].c);
//        }
//    }
//
    for (int i = 0; i < masses.size(); i++) {
        glm::dvec3 currentForce(0.0);
        double F_c = 0;
        double F_h = 0;
        double F_v = 0;
        for (const Spring spring: springs) {
//            cout << "springLength: " << spring.L0 << endl;
            if(spring.m1 == i || spring.m2 == i) {
//                cout << springs[i].m1 << " " << springs[i].m2 << endl;
                glm::dvec3 springForce(0.0);
                glm::dvec3 direction(0.0);
                direction = glm::normalize(masses[spring.m2].p - masses[spring.m1].p);
                double currentLength = distance(masses[spring.m1].p, masses[spring.m2].p);
//                cout <<"current Length: " << currentLength <<  endl;
                double F = spring.k * (currentLength - spring.L0);
//                cout << "current f" << F << endl;
//                cout << "direction " << direction.x << endl;
                if(spring.m1 == i) {
                    springForce = F * direction;
                }
                else {
                    springForce = -F * direction;
                }
                currentForce += + springForce;
            }
        }
        currentForce.z = currentForce.z + GRAVITY.z * MASS;
        if (masses[i].p[2] < 0) {
            F_c = -kGround * masses[i].p[2];
            cout << "fc" << F_c << endl;
            currentForce.z = currentForce.z + F_c;
            F_h = sqrt(pow(currentForce.x, 2) +  pow(currentForce.y, 2));
            F_v = currentForce.z;
            if (F_h < mu * F_v) {
                masses[i].p[0] = 0;
                masses[i].p[1] = 0;
            }
            else {
                currentForce.x = currentForce.x - F_v * mu;
                currentForce.y = currentForce.y - F_v * mu;
            }
        }

        masses[i].v *= DAMPING;
        masses[i].a = currentForce/MASS;
        masses[i].v += masses[i].a * dt;
        masses[i].p += masses[i].v * dt;
    }
    T += dt;
}


// renew indices!
void updateVertices() {
    for (int i = 0; i < masses.size(); i++) {
        for (int j = 0; j < 3; j++) {
            cubeVertices[3 * i + j] = masses[i].p[j];
        }
    }
}

void someStuffToMakesuretheDrawingWroking() {
    for (int i = 0; i < DIM*DIM*DIM; i++) {
        vector<int> tempCube(8);
        vector<int> upper(0);
        vector<int> down(0);
        vector<int> front(0);
        vector<int> back(0);
        int right;
        int left;
        for (int j = 0; j < 8; j++) {
            tempCube[j] = imMasses[8 * i + j];
        }
        for (int n = 0; n < 8; n++) {
            if (masses[tempCube[n]].p.z == masses[tempCube[0]].p.z){
                upper.push_back(tempCube[n]);
            }
            else {
                down.push_back(tempCube[n]);
            }
        }
        cout << upper[0]<<  endl;
        // change position if wrong
        if (masses[upper[0]].p.z < masses[down[0]].p.z) {
            vector<int> temp = upper;
            upper = down;
            down = temp;
        }
        
        for (int n = 0; n < 4; n++) {
            if (masses[upper[n]].p.y == masses[upper[0]].p.y) {
                front.push_back(upper[n]);
            }
            else {
                back.push_back(upper[n]);
            }
        }
//        cout << "front: " << front.size() << endl;
//        cout << "back: " << back.size() << endl;
        // change the position if wrong
        if (masses[front[0]].p.y < masses[back[0]].p.y) {
            vector<int> temp = front;
            front = back;
            back = temp;
        }
//
        if (masses[front[0]].p.x > masses[front[1]].p.x) {
            right = front[0];
            left = front[1];
        }
        else {
            right = front[1];
            left = front[0];
        }
        tempCube[5] = right;
        tempCube[6] = left;
        for (int z = 0; z < 4; z++) {
            if(masses[upper[z]].p.x == masses[right].p.x && masses[upper[z]].p.y < masses[right].p.y) {
                tempCube[4] = upper[z];
            }
            if(masses[upper[z]].p.x < masses[right].p.x && masses[upper[z]].p.y < masses[right].p.y) {
                tempCube[7] = upper[z];
            }
        }
        for (int p = 0; p < 4; p++) {
            if(masses[down[p]].p.x == masses[tempCube[4]].p.x && masses[down[p]].p.y == masses[tempCube[4]].p.y){
                tempCube[0] = down[p];
            }
            if(masses[down[p]].p.x == masses[tempCube[5]].p.x && masses[down[p]].p.y == masses[tempCube[5]].p.y){
                tempCube[1] = down[p];
            }
            if(masses[down[p]].p.x == masses[tempCube[6]].p.x && masses[down[p]].p.y == masses[tempCube[6]].p.y){
                tempCube[2] = down[p];
            }
            if(masses[down[p]].p.x == masses[tempCube[7]].p.x && masses[down[p]].p.y == masses[tempCube[7]].p.y){
                tempCube[3] = down[p];
            }
        }
        for (int j = 0; j < 8; j++) {
            imMasses[8 * i + j] = tempCube[j];
        };
        
        for (int k = 0; k < 12; k++) {
            cubeIndices[36 * i + 3 * k] = tempCube[faceIndices[k][0]];
            cubeIndices[36 * i + 1 + 3 * k] = tempCube[faceIndices[k][1]];
            cubeIndices[36 * i + 2 + 3 * k] = tempCube[faceIndices[k][2]];
        }
        
        for (int g = 0; g < 12; g++) {
            myEdge_indices[24 * i + 2 * g] = tempCube[edgeIndices[g][0]];
            myEdge_indices[24 * i + 2 * g + 1] = tempCube[edgeIndices[g][1]];
        }
    }
}
