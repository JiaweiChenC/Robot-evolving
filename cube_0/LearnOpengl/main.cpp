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
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 1000;

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
int robot_num = 1;

// timing
float deltaTime = 0.0f;    // time between current frame and last frame
float lastFrame = 0.0f;
// settings
double a = LENGTH;
double b = 0.1;
double c = 0.1;


bool animation = true;

extern vector<GLdouble> cubeColor;
extern vector<GLdouble> pointColor;
extern vector<GLdouble> cubeVertices;
//GLdouble cubeColor[mass_num * 3 * robot_num]; // cube_count * 64 points * 3 bit
extern vector<GLuint> cubeIndices;
// cube_count * 6 faces * 2 triangles * 3 indices
extern vector<GLdouble> myEdge_color;      // cube_count * 8 points * 3 bit
extern vector<GLuint> myEdge_indices; // cube_count * 12 * 2

vector<vector<GLuint>> cubeIndicesGroup;
vector<vector<GLuint>> edgeIndicesGroup;
vector<vector<GLdouble>> cubeVerticesGroup;
vector<vector<GLdouble>>  cubeColorGroup;
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


int main()
{
    srand(time(NULL));
//    vector<Robot> robots = geneticAlgorithm(10, 1, 10, 2, true);
    
    Robot robot0(0, 0.0, 0.0,20);
    Robot robot1(-1, 0.0, 0.0, 20);
    Robot robot2(-2, 0.0, 0.0, 20);
    Robot robot3(-3, 0.0, 0.0, 20);
    Robot robot4(-4, 0.0, 0.0, 20);
    Robot robot5(1, 0.0, 0.0, 20);
    Robot robot6(2, 0.0, 0.0, 20);
    Robot robot7(3, 0.0, 0.0, 20);
    Robot robot8(4, 0.0, 0.0, 20);
    
    ifstream inData;
    vector<vector<double>> cubes(28, vector<double> (3));
    inData.open("/Users/jiaweichen/Desktop/data.txt");
    for (int i=0; i<28; i++) {
    inData >> cubes[i][0] >> cubes[i][1] >> cubes[i][2];
    }
    inData.close();
    cout << cubes.size() << endl;
    Robot robotz(cubes);
    robot0 = robotz;
//    vector<vector<double>> pos;
//    pos.push_back(p1);
//    pos.push_back(p2);
//    pos.push_back(p3);
//    pos.push_back(p4);
//    robot0  = deleteCubes(robot0, pos);
//    for (int i = 0; i < 12 ; i++)
//    {robot1 = mutateRobot(robot1);}
//    vector<Robot> child = crossoverRobot(robot0, robot1);
//    robot1 = child[0];
//    robot0 = child[1];
    cout << "size: " << robot1.cubes.size() << endl;
    vector<Robot> robotGroup;
    robotGroup.push_back(robot0);
    robotGroup.push_back(robot1);
    robotGroup.push_back(robot2);
    robotGroup.push_back(robot3);
    robotGroup.push_back(robot4);
    robotGroup.push_back(robot5);
    robotGroup.push_back(robot6);
    robotGroup.push_back(robot7);
    robotGroup.push_back(robot8);

//    int num = 0;
//    for (const auto& cube: robotGroup) {
//        cout << "robot " << num << ": " << endl;
//        for(const auto& position: cube.existCube) {
//            cout << position[0] << " " << position[1] << " " << position[2] << endl;
//            num += 1;
//        }
//    }
//    for (const auto& robot: robotGroup) {
//        cout << robot.masses.size() << endl;
//    }
//    cout << "masses.size" << robot0.masses.size() << endl;
//    cout << "cube_num :" <<robot0.cube_num << endl;
//    cout << "color size" << cubeCoslor.size() << endl;
//    cout << "masses indices" << robot0.imMasses.size() << endl;
//    for (const auto& i: robot1.masses) {
//        cout << i.p[0] << " " << i.p[1] << " " << i.p[2] << " " << endl;
//    }
//    for (const auto& i: robot0.existCube) {
//        cout << "exist: " << i[0] << " " << i[1]  << " " << i[2] <<   " " << endl;
//    }
//    for (int i = 0; i < robot0.imMasses.size(); i++) {
//        if(i % 8 == 0) {
//            cout << endl;
//        }
//        cout << robot0.imMasses[i] << " ";
//    }
    for (int i = 0; i < robotGroup.size(); i++) {
        robotGroup[i].updateVertices();
        robotGroup[i].someStuffToMakesuretheDrawingWroking();
        cubeIndicesGroup.push_back(cubeIndices);
        edgeIndicesGroup.push_back(myEdge_indices);
        cubeVerticesGroup.push_back(cubeVertices);
        cubeColorGroup.push_back(cubeColor);
        cubeColor.clear();
        cubeIndices.clear();
        myEdge_indices.clear();
    }
    
    
    cout << "group size: " << cubeIndicesGroup.size() << endl;
    cout <<" _____________ "<<cubeIndices.size() << endl;
    cout << "cube Indices.size: " << cubeIndices.size() << endl;
    cout << "size: " << cubeIndices.size() << endl;
    cout << "vertices: " << cubeVertices.size() << endl;
    cout << "size is " << robot0.masses.size() << endl;
    //    for (int i = 0; i < robot0.masses.size(); i++) {
    //        cout << "position" << robot0.masses[i].p[0] << ", " << robot0.masses[i].p[1] <<", " << robot0.masses[i].p[2] << endl;
    //    }

    //    robot0.runningSimulate(2);
    if (animation){
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
        //unsigned int cubeVAO, cubeVBO, cubeEBO;
        vector<GLuint> cubeVAO(9);
        vector<GLuint> cubeVBO(9);
        vector<GLuint> cubeEBO(9);
        for (int i = 0; i < 9; i++) {
            glGenVertexArrays(1, &cubeVAO[i]);
            glGenBuffers(1, &cubeVBO[i]);
            glGenBuffers(1, &cubeEBO[i]);
            glBindVertexArray(cubeVAO[i]);
            glBindBuffer(GL_ARRAY_BUFFER, cubeVBO[i]);
            assert(cubeVertices.size() != 0);
//            cout << "i: " << cubeVertices.size() << endl;
//            cout << "I: " << cubeVertices.data() << endl;

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO[i]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, cubeIndicesGroup[i].size() * 4, cubeIndicesGroup[i].data(), GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
            glEnableVertexAttribArray(0);
            GLuint colorbufferCube;
            glGenBuffers(1, &colorbufferCube);
            glBindBuffer(GL_ARRAY_BUFFER, colorbufferCube);
            glBufferData(GL_ARRAY_BUFFER, cubeColorGroup[i].size() * 8, cubeColorGroup[i].data(), GL_STATIC_DRAW);
            glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
            glEnableVertexAttribArray(1);
        }
        
        Shader edgeShader("cube_0/lineShader.vs", "cube_0/lineShader.fs");
        
        vector<GLuint> edgeVAO(9);
        vector<GLuint> edgeVBO(9);
        vector<GLuint> edgeEBO(9);
        for (int i = 0; i < 9; i++) {
            glGenVertexArrays(1, &edgeVAO[i]);
            glGenBuffers(1, &edgeVBO[i]);
            glGenBuffers(1, &edgeEBO[i]);
            glBindVertexArray(edgeVAO[i]);
            glBindBuffer(GL_ARRAY_BUFFER, edgeVBO[i]);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edgeEBO[i]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, edgeIndicesGroup[i].size() * 4, edgeIndicesGroup[i].data(), GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
            glEnableVertexAttribArray(0);
        }

        Shader pointShader("cube_0/pointShader.vs", "cube_0/pointShader.fs");
        unsigned int pointVAO, pointVBO;
        glGenVertexArrays(1, &pointVAO);
        glGenBuffers(1, &pointVBO);
        glBindVertexArray(pointVAO);
        glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
        glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
        glEnableVertexAttribArray(0);
        GLuint pointColorBuffer;
        glGenBuffers(1, &pointColorBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, pointColorBuffer);
        glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, (void*)0); // set to 0, opengl decide
        glEnableVertexAttribArray(1);


    // cubeColor buffer
    while (!glfwWindowShouldClose(window))
    {
        // input
        // -----
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
            
//            robot0.runningSimulate(2);
//            robot0.setDistance();
            T += dt;
            cout << "global Time: " << T << endl;
//            cout << "global time: " << T << endl;
//            cout <<"at height: " << robot0.getPosition().z << endl;
//            cout <<"travel distance" << robot0.getDistance() << endl;
            glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            // std::cout << "Global Time now: " << T << endl;

            glDepthFunc(GL_LESS);
            planeShader.use();
            planeShader.setInt("texture1", 0);
            model = glm::mat4(1.0f);
            model = glm::translate(model, vec3(0.5, -1, 0.5));
            planeShader.setMat4("model",  model);
            planeShader.setMat4("view", view);
            planeShader.setMat4("projection", projection);
            glBindVertexArray(planeVAO);
            glBindTexture(GL_TEXTURE_2D, floorTexture);
            glDrawArrays(GL_TRIANGLES, 0, 6);
            glBindVertexArray(0);
//
            model = glm::rotate(model, glm::radians(270.0f), glm::vec3(1, 0, 0));
            model = glm::translate(model, vec3(0.5, -0.5, -0.5));

            // robot 0
            for (int i = 0; i < 1; i++) {
                robotGroup[i].updateRobot();
                robotGroup[i].updateVertices();
                robotGroup[i].breathing();
                
                glDepthFunc(GL_ALWAYS);
                cubeShader.use();
                cubeShader.setMat4("MVP", projection * view * model);
                glBindVertexArray(cubeVAO[i]);
                glBindBuffer(GL_ARRAY_BUFFER, cubeVBO[i]);
                glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * 8, cubeVertices.data(), GL_DYNAMIC_DRAW);
                glDrawElements(GL_TRIANGLES, 12 * 3 * robotGroup[i].cube_num, GL_UNSIGNED_INT, 0); // 12 triangle 3 points 27 cube
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
                
                glDepthFunc(GL_ALWAYS);
                edgeShader.use();
                edgeShader.setMat4("MVP", projection * view * model);
                glBindVertexArray(edgeVAO[i]);
                glBindBuffer(GL_ARRAY_BUFFER, edgeVBO[i]);
                glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * 8, cubeVertices.data(), GL_DYNAMIC_DRAW);
                glDrawElements(GL_LINES, 12 * 2 * robotGroup[i].cube_num, GL_UNSIGNED_INT, 0); // 12 triangle 3 points 27 cube
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
                
                glDepthFunc(GL_LESS);
                glPointSize(7.0f);
                pointShader.use();
                pointShader.setMat4("MVP", projection * view * model);
                glBindVertexArray(pointVAO);
                glBindBuffer(GL_ARRAY_BUFFER, pointColorBuffer);
                glBufferData(GL_ARRAY_BUFFER, pointColor.size() * 8, pointColor.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
                glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * 8, cubeVertices.data(), GL_DYNAMIC_DRAW);
                glDrawArrays(GL_POINTS, 0, (int)robotGroup[i].masses.size());// 12 triangle 3 points 27 cube
                glBindBuffer(GL_ARRAY_BUFFER, 0);
                glBindVertexArray(0);
                
//                cout << i << "  "<<  robotGroup[i].getPosition()[0]<< " " << robotGroup[i].getPosition()[1] <<" " <<  robotGroup[i].getPosition()[2] << endl;
                cubeVertices.clear();
                pointColor.clear();
                cubeColor.clear();
            }
            
            
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

