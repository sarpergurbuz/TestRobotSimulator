//
// Created by sarper on 14.07.25.
//
#include <iostream>
#include <iostream>
#include <vector>
#include <memory>
#include <AndreiUtils/utilsMap.hpp>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsString.h>
#include <ConceptLibrary/entities/ObjectConcept.h>
#include <ConceptLibrary/instances/geometry/BoxShape.h>
#include <ConceptLibrary/instances/geometry/PyramidShape.h>
#include <ConceptLibrary/instances/ObjectInstanceGeometry.h>
#include <ConceptLibrary/instances/utils.h>
#include <ConceptLibrary/valueDomains/GeometricShape.h>
#include <ConceptLibrary/valueDomains/InstanceBase.h>
#include <ConceptLibrary/valueDomains/Location.h>
#include <ConceptLibrary/valueDomains/Pose.h>
#include <ConceptLibrary/LibraryInit.h>
#include <ConceptLibrary/utils.h>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

using namespace ConceptLibrary;

class MotionPrimitiveParam {
public:
    virtual ~MotionPrimitiveParam() = default;
};

class MoveRobotBodyCartesianParam : public MotionPrimitiveParam {
public:
    Pose goalPose;

    explicit MoveRobotBodyCartesianParam(Pose pose) : MotionPrimitiveParam(), goalPose(std::move(pose)) {}
};

class MoveGripperParam : public MotionPrimitiveParam {
public:
    bool openGripper;

    explicit MoveGripperParam(bool isOpen) : MotionPrimitiveParam(), openGripper(isOpen) {}
};

class SetObjectInGripperParam : public MotionPrimitiveParam {
public:
    InstanceAccept<ObjectConcept> o;

    explicit SetObjectInGripperParam(Instance<ConceptList<ObjectConcept>> const &objectInstance) :
            MotionPrimitiveParam(), o(objectInstance) {}
};

// Motion Primitive Executors
void executeMoveRobotBodyCartesian(const MoveRobotBodyCartesianParam& param,Robot* robot, PathController& path) {


    path.simulationControlToDestination(robot, fromPoseToDQ(param.goalPose));
    AndreiUtils::sleepMSec(1000);
}

void executeSetObjectInGripper(const SetObjectInGripperParam& param, Robot* robot,SimulationInterface& sim, const std::map<std::string, std::string>& nameConvertor) {
    Gripper* gripper = robot->getGripper();
    const std::string& instanceId = param.o->instanceId.s;
    const std::string& simObjectName = AndreiUtils::mapGet(nameConvertor, instanceId);

    sim.get().set_object_parent(simObjectName, gripper->getGripperName(), true);
    AndreiUtils::sleepMSec(1000);
}



void dispatchMotionPrimitives(const std::vector<std::shared_ptr<MotionPrimitiveParam>>& primitives) {   // NOTE: maybe take robot and path as input  :   , Robot* robot,PathController& path

    SimulationInterface &sim = SimulationInterface::getInterface();
    Robot robot(Robot::createFromType("PandaRobot", sim, "PandaGripper"));
    PathController path;

    for (const auto& primitive : primitives) {
        if (auto moveBody = std::dynamic_pointer_cast<MoveRobotBodyCartesianParam>(primitive)) {
            executeMoveRobotBodyCartesian(*moveBody, &robot, path);

        } else if (auto gripper = std::dynamic_pointer_cast<MoveGripperParam>(primitive)) {
            std::cout << "[Dispatch] Executing MoveGripper → "
                      << "Open: " << std::boolalpha << gripper->openGripper << "\n";

        } else if (auto setObj = std::dynamic_pointer_cast<SetObjectInGripperParam>(primitive)) {
            executeSetObjectInGripper(*setObj, &robot, sim, nameConvertor);

        } else {
            std::cout << "[Dispatch] Unknown motion primitive type\n";
        }
    }
}




class Execution {
public:
    // Constructor initialized once
    Execution()
        : sim(SimulationInterface::getInterface()),
          robot(Robot::createFromType("PandaRobot", sim, "PandaGripper")),
          path()
    {
        initializeSimData(nameConvertor, objectSpecificTranslationAdjustment);
    }

    // Method to execute a single motion primitive
    void executability(const std::shared_ptr<MotionPrimitiveParam>& primitive) {
        if (auto moveBody = std::dynamic_pointer_cast<MoveRobotBodyCartesianParam>(primitive)) {
            executeMoveRobotBodyCartesian(*moveBody);
        } else if (auto gripper = std::dynamic_pointer_cast<MoveGripperParam>(primitive)) {
            std::cout << "[Execution] Executing MoveGripper → "
                      << "Open: " << std::boolalpha << gripper->openGripper << "\n";
        } else if (auto setObj = std::dynamic_pointer_cast<SetObjectInGripperParam>(primitive)) {
            executeSetObjectInGripper(*setObj);
        } else {
            std::cout << "[Execution] Unknown motion primitive type\n";
        }
    }

private:
    // Internally held system objects
    SimulationInterface& sim;
    Robot robot;
    PathController path;

    // One-time initialized mappings
    std::map<std::string, std::string> nameConvertor;
    std::map<std::string, Eigen::Vector3d> objectSpecificTranslationAdjustment;

    // Internal executors
    void executeMoveRobotBodyCartesian(const MoveRobotBodyCartesianParam& param) {
        path.simulationControlToDestination(&robot, fromPoseToDQ(param.goalPose));
        AndreiUtils::sleepMSec(1000);
    }

    void executeSetObjectInGripper(const SetObjectInGripperParam& param) {
        Gripper* gripper = robot.getGripper();
        const std::string& instanceId = param.o->instanceId.s;
        const std::string& simObjectName = AndreiUtils::mapGet(nameConvertor, instanceId);

        sim.get().set_object_parent(simObjectName, gripper->getGripperName(), true);
        AndreiUtils::sleepMSec(1000);
    }
};





// Changing the interface

class Execution : public ConceptLibrary::Executor {
public:
    Execution()
        : Executor("Execution"),  // Name of the domain
          sim(SimulationInterface::getInterface()),
          robot(Robot::createFromType("PandaRobot", sim, "PandaGripper")),
          path() {
        initializeSimData(nameConvertor, objectSpecificTranslationAdjustment);
        // Register abilities that this class can handle
        addAbility("MoveRobotBodyCartesian");
        addAbility("MoveGripper");
        addAbility("SetObjectInGripper");
    }

    bool executeAbility(InstanceAccept<AbilityConcept> const &ability) override {
        std::string const &abilityName = ability.name;

        if (abilityName == "MoveRobotBodyCartesian") {
            AndreiUtils::Pose goal = samplePose(0.2, false, true);  // placeholder
            executeMoveRobotBodyCartesian(MoveRobotBodyCartesianParam(goal));
        } else if (abilityName == "MoveGripper") {
            executeMoveGripper(MoveGripperParam(true));  // hardcoded for now
        } else if (abilityName == "SetObjectInGripper") {
            executeSetObjectInGripper(SetObjectInGripperParam(InstanceAccept<ObjectConcept>("MilkCartonLidlInstance1")));
        } else {
            std::cout << "[Execution] Unknown ability name: " << abilityName << std::endl;
            return false;
        }

        return true;
    }

private:
    SimulationInterface& sim;
    Robot robot;
    PathController path;
    std::map<std::string, std::string> nameConvertor;
    std::map<std::string, Eigen::Vector3d> objectSpecificTranslationAdjustment;

    void executeMoveRobotBodyCartesian(MoveRobotBodyCartesianParam const &param) {
        std::cout << "Executing: MoveRobotBodyCartesian\n";
        path.simulationControlToDestination(&robot, fromPoseToDQ(param.goalPose));
        AndreiUtils::sleepMSec(1000);
    }

    void executeMoveGripper(MoveGripperParam const &param) {
        std::cout << "Executing: MoveGripper (open: " << param.openGripper << ")\n";
        // Insert actual gripper command if needed
        AndreiUtils::sleepMSec(1000);
    }

    void executeSetObjectInGripper(SetObjectInGripperParam const &param) {
        std::cout << "Executing: SetObjectInGripper\n";
        Gripper* gripper = robot.getGripper();
        std::string const &instanceId = param.o.instanceId.s;
        std::string const &simObjectName = AndreiUtils::mapGet(nameConvertor, instanceId);
        sim.get().set_object_parent(simObjectName, gripper->getGripperName(), true);
        AndreiUtils::sleepMSec(1000);
    }
};

void testConceptAbilityExecution() {
    Execution exec;

    std::vector<InstanceAccept<AbilityConcept>> abilities = {
        InstanceAccept<AbilityConcept>("MoveRobotBodyCartesian"),
        InstanceAccept<AbilityConcept>("MoveGripper"),
        InstanceAccept<AbilityConcept>("SetObjectInGripper")
    };

    for (auto const &ability : abilities) {
        exec.executeAbility(ability);
    }
}


