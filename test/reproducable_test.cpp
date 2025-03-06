#include "../src/WorldPhysics.hpp"
#include "../src/objects/ObjectFactory.hpp"
#include "../src/objects/Box.hpp"
#include "../src/objects/Sphere.hpp"
#include "../src/objects/Inertial.hpp"
#include "../src/joints/JointFactory.hpp"
#include "../src/joints/HingeJoint.hpp"
#include "../src/joints/FixedJoint.hpp"
#include "../src/joints/SliderJoint.hpp"

#include <cstdio>

#define LOG_ERROR(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
#define LOG_WARN(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
#define LOG_DEBUG(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")
#define LOG_INFO(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n")

using namespace mars::ode_physics;
using namespace mars::utils;
using namespace std;
using namespace configmaps;

WorldPhysics *worldPhysics = NULL;

shared_ptr<DynamicObject> createBox(ConfigMap &map, string name, Vector extend, Vector pos, double mass)
{
    map["name"] << name;
    map["parentFrame"] << name;
    map["type"] = "box";
    map["extend"]["x"] = extend.x();
    map["extend"]["y"] = 0.2;
    map["extend"]["z"] = 0.05;
    map["position"]["x"] = 0.0;
    map["position"]["y"] = 0.0;
    map["position"]["z"] = 0.265;
    map["orientation"]["x"] = 0.0;
    map["orientation"]["y"] = 0.0;
    map["orientation"]["z"] = 0.0;
    map["orientation"]["w"] = 1.0;
    map["density"] = 1.0;
    map["mass"] = mass;
    //LOG_ERROR("create box: %s", m.toYamlString().c_str());
    LOG_ERROR("worldPhysics: %p", worldPhysics);
    //LOG_ERROR("map: %p", &m);
    shared_ptr<DynamicObject> frame = worldPhysics->createFrame(NULL, map);
    worldPhysics->createObject(map);
    return frame;
}

int main(int argc, char *argv[])
{

    ObjectFactory::Instance().addObjectType("box", &Box::instantiate);
    ObjectFactory::Instance().addObjectType("sphere", &Box::instantiate);
    ObjectFactory::Instance().addObjectType("inertial", &Inertial::instantiate);
    JointFactory::Instance().addJointType("hinge", &HingeJoint::instantiate);
    JointFactory::Instance().addJointType("fixed", &FixedJoint::instantiate);
    JointFactory::Instance().addJointType("slider", &SliderJoint::instantiate);
    JointFactory::Instance().addJointType("prismatic", &SliderJoint::instantiate);

    // create world
    worldPhysics = new WorldPhysics();
    LOG_ERROR("worldPhysics: %p", worldPhysics);
    ConfigMap map;
    map["name"] = "da";
    // worldPhysics->test(2);
    // worldPhysics->test2(map);
    // worldPhysics->createFrame(NULL, map);
    worldPhysics->initTheWorld();
    worldPhysics->step_size = 0.001;
    worldPhysics->fast_step = true;

    // create bodies
    vector<shared_ptr<DynamicObject>> bodyList;
    vector<shared_ptr<JointInterface>> jointList;
    vector<shared_ptr<DynamicObject>> feetList;
    shared_ptr<DynamicObject> body;
    Vector pos(0.0, 0.0, 0.265);

    // shared_ptr within this block are released afterwards
    {
        shared_ptr<DynamicObject> object;
        Vector extend(0.4, 0.2, 0.05);
        body = createBox(map, "body", extend, pos, 1.0);
        bodyList.push_back(body);

        extend = Vector(0.03, 0.03, 0.2);
        pos = Vector(0.2-0.015, 0.1-0.015, 0.34);
        object = createBox(map, "fl_upper", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.z() = 0.14;
        object = createBox(map, "fl_lower", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.y() = -(0.1-0.0015);
        pos.z() = 0.34;
        object = createBox(map, "fr_upper", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.z() = 0.14;
        object = createBox(map, "fr_lower", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.x() = -(0.2-0.0015);
        pos.y() = (0.1-0.0015);
        pos.z() = 0.34;
        object = createBox(map, "rl_upper", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.z() = 0.14;
        object = createBox(map, "rl_lower", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.y() = -(0.1-0.0015);
        pos.z() = 0.34;
        object = createBox(map, "rr_upper", extend, pos, 0.2);
        bodyList.push_back(object);

        pos.z() = 0.14;
        object = createBox(map, "rr_lower", extend, pos, 0.2);
        bodyList.push_back(object);

        map["type"] << "sphere";
        map["mass"] = 0.05;
        map["extend"]["x"] = 0.02;
        map["position"]["z"] = 0.02;
        map["position"]["x"] = (0.2-0.0015);
        map["position"]["y"] = (0.1-0.0015);
        map["name"] << "fl_foot";
        map["parentFrame"] << "fl_foot";
        shared_ptr<DynamicObject> frame = worldPhysics->createFrame(NULL, map);
        feetList.push_back(frame);
        worldPhysics->createObject(map);
        map["position"]["y"] = -(0.1-0.0015);
        map["name"] << "fr_foot";
        map["parentFrame"] << "fr_foot";
        frame = worldPhysics->createFrame(NULL, map);
        feetList.push_back(frame);
        worldPhysics->createObject(map);
        map["position"]["x"] = -(0.2-0.0015);
        map["position"]["y"] = (0.1-0.0015);
        map["name"] << "rl_foot";
        map["parentFrame"] << "rl_foot";
        frame = worldPhysics->createFrame(NULL, map);
        feetList.push_back(frame);
        worldPhysics->createObject(map);
        map["position"]["y"] = -(0.1-0.0015);
        map["name"] << "rr_foot";
        map["parentFrame"] << "rr_foot";
        frame = worldPhysics->createFrame(NULL, map);
        feetList.push_back(frame);
        worldPhysics->createObject(map);

        // create joints
        shared_ptr<JointInterface> joint;
        ConfigMap jointMap;
        jointMap["name"] << "fl_hip";
        jointMap["type"] << "hinge";
        jointMap["parent_link_name"] << "body";
        jointMap["child_link_name"] << "fl_upper";
        jointMap["anchor"]["x"] = 0.2-0.015;
        jointMap["anchor"]["y"] = 0.1-0.015;
        jointMap["anchor"]["z"] = 0.44;
        jointMap["axis1"]["x"] = 0.0;
        jointMap["axis1"]["y"] = 1.0;
        jointMap["axis1"]["z"] = 0.0;
        jointMap["reducedDataPackage"] = true;
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "fr_hip";
        jointMap["child_link_name"] << "fr_upper";
        jointMap["anchor"]["y"] = -(0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "rl_hip";
        jointMap["child_link_name"] << "rl_upper";
        jointMap["anchor"]["x"] = -(0.2-0.015);
        jointMap["anchor"]["y"] = (0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "rr_hip";
        jointMap["child_link_name"] << "rr_upper";
        jointMap["anchor"]["y"] = -(0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "fl_knee";
        jointMap["parent_link_name"] << "fl_upper";
        jointMap["child_link_name"] << "fl_lower";
        jointMap["anchor"]["x"] = (0.2-0.015);
        jointMap["anchor"]["y"] = (0.1-0.015);
        jointMap["anchor"]["z"] = 0.24;
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "fr_knee";
        jointMap["parent_link_name"] << "fr_upper";
        jointMap["child_link_name"] << "fr_lower";
        jointMap["anchor"]["y"] = -(0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "rl_knee";
        jointMap["parent_link_name"] << "rl_upper";
        jointMap["child_link_name"] << "rl_lower";
        jointMap["anchor"]["x"] = -(0.2-0.015);
        jointMap["anchor"]["y"] = (0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "rr_knee";
        jointMap["parent_link_name"] << "rr_upper";
        jointMap["child_link_name"] << "rr_lower";
        jointMap["anchor"]["y"] = -(0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "fl_ankle";
        jointMap["type"] << "fixed";
        jointMap["parent_link_name"] << "fl_lower";
        jointMap["child_link_name"] << "fl_foot";
        jointMap["anchor"]["x"] = 0.2-0.015;
        jointMap["anchor"]["y"] = 0.1-0.015;
        jointMap["anchor"]["z"] = 0.4;
        jointMap["reducedDataPackage"] = true;
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "fr_ankle";
        jointMap["parent_link_name"] << "fr_lower";
        jointMap["child_link_name"] << "fr_foot";
        jointMap["anchor"]["y"] = -(0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "rl_ankle";
        jointMap["parent_link_name"] << "rl_lower";
        jointMap["child_link_name"] << "rl_foot";
        jointMap["anchor"]["x"] = -(0.2-0.015);
        jointMap["anchor"]["y"] = (0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);

        jointMap["name"] << "rr_ankle";
        jointMap["parent_link_name"] << "rr_lower";
        jointMap["child_link_name"] << "rr_foot";
        jointMap["anchor"]["y"] = -(0.1-0.015);
        joint = worldPhysics->createJoint(NULL, jointMap);
        jointList.push_back(joint);
    }

    for(auto &it: jointList)
    {
        it->setForceLimit(9);
        it->setJointAsMotor(1);
    }

    double time = 0.0;
    double hip_value = 0.0;
    double knee_value = 0.0;
    double offset[4] = {0.0, 1.14, 0.0, 1.14};
    FILE *file = fopen("reproducable_test_log.csv", "w");
    fprintf(file, "time_sec,body_x,body_y,body_z\n");

    // simulate for seconds sine pattern and log body position to file
    {
        ContactData cd;

        cd.body1 = nullptr;
        cd.body2 = nullptr;
        cd.c_params.cfm = 0.00001;
        cd.c_params.erp = 0.2;
        cd.c_params.friction1 = 0.8;
        cd.c_params.friction2 = 0.8;
        cd.nameObject1 = "";
        cd.nameObject2 = "";
        while(time < 4.0)
        {
            body->getPosition(&pos);
            fprintf(file, "%g,%g,%g,%g\n", time, pos.x(), pos.y(), pos.z());

            hip_value = 0.5*sin(time*4*M_PI)-0.3;
            knee_value = -0.5*sin(1+time*4*M_PI)+0.3;
            for(int i=0; i<4; i++)
            {
                // position controller with p: 20
                hip_value = 0.5*sin(offset[i]+time*4*M_PI)-0.3;
                knee_value = -0.5*sin(offset[i]+1+time*4*M_PI)+0.3;
                jointList[i]->setVelocity((hip_value - jointList[i]->getPosition())*20);
                jointList[i+4]->setVelocity((knee_value - jointList[i+4]->getPosition())*20);
            }
            worldPhysics->clearPreviousStep();
            for(auto &foot: feetList)
            {
                foot->getPosition(&pos);
                pos.z() -= 0.02;
                if(pos.z() <= 0.0) {
                    cd.pos = pos;
                    cd.pos.z() = 0.0;
                    cd.depth = pos.z();
                    cd.normal = Vector(0.0, 0.0, 1.0);
                    cd.body1 = foot;
                    cd.nameObject1 = foot->getName();
                    foot->addContact(cd);
                }
            }
            worldPhysics->stepTheWorld();
            time += worldPhysics->step_size;
        }
        fclose(file);
    }

    // clean up
    jointList.clear();

    for(auto &it: bodyList)
    {
        worldPhysics->removeFrame(it->getName());
    }
    bodyList.clear();
    body = nullptr;
    for(auto &it: feetList)
    {
        worldPhysics->removeFrame(it->getName());
    }
    feetList.clear();
    worldPhysics->freeTheWorld();
    return 0;
}
