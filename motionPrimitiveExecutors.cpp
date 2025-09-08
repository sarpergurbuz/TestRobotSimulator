//
// Created by sarper on 14.07.25.
//




#include <AndreiUtils/utilsMap.hpp>

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
#include <AndreiUtils/classes/graph/Graph.hpp>
#include <AndreiUtils/classes/RandomNumberGenerator.hpp>
#include <AndreiUtils/classes/PoseDecoupledInterpolator.hpp>
#include <AndreiUtils/classes/Timer.hpp>
#include <AndreiUtils/utilsEigenGeometry.h>
#include <AndreiUtils/utilsEigenGeometry.hpp>
#include <AndreiUtils/utilsGeometry.h>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsJsonEigen.hpp>
#include <AndreiUtils/utilsPose.hpp>
#include <AndreiUtils/utilsThread.h>
#include <ConceptLibrary/abilities/MoveRobotBodyCartesianAbility.h>
#include <ConceptLibrary/abilities/MoveGripperAbility.h>
#include <ConceptLibrary/abilities/SetObjectInGripperAbility.h>
#include <ConceptLibrary/abilities/MoveRobotBodyCartesianWithIntermediateGoalsAbility.h>
#include <ConceptLibrary/abilities/ClearObjectInGripperAbility.h>
#include <ConceptLibrary/abilities/SeeThenMoveToObjectAbility.h>
#include <ConceptLibrary/entities/FrankaRobotConcept.h>
#include <ConceptLibrary/entities/OpenableObjectConcept.h>
#include <ConceptLibrary/entities/PourerConcept.h>
#include <ConceptLibrary/entities/PourerSurfaceConcept.h>
#include <ConceptLibrary/entities/PouringSurfaceConcept.h>
#include <ConceptLibrary/functions/EnvironmentDataFunctions.h>
#include <ConceptLibrary/instances/utils.h>
#include <ConceptLibrary/LibraryInit.h>
#include <ConceptLibrary/skills/GraspSkill.h>
#include <ConceptLibrary/skills/TransportSkill.h>
#include <ConceptLibrary/skills/ReleaseSkill.h>
#include <ConceptLibrary/skills/PourSkill.h>
#include <ConceptLibrary/utils.h>
#include <ConceptLibrary/valueDomains/variations/utilsJson.hpp>
#include <iostream>
#include <PerceptionData/detections/ObjectTagModelDetection.h>
#include <RobotModelling/PathController.h>
#include <RobotModelling/Robot.h>
#include <RobotModelling/SimulationInterface.h>
#include <RobotModelling/SimulationObjectRelativeGrasp.h>
#include <RobotModelling/utils.h>
#include <ConceptLibrary/valueDomains/Pose.h>
#include <limits>
#include <unordered_set>

using namespace ConceptLibrary;
using namespace AndreiUtils;
using namespace ConceptLibrary;
using namespace DQ_robotics;
using namespace Eigen;
using namespace std;
using namespace PerceptionData;
using namespace RobotModelling;

/*
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

*/



// ----------------------------------BT FUNCTIONS

Instance<ConceptList<PouringSurfaceConcept>> DeterminePouringSurface(Instance<ConceptList<ContainerConcept>> const &into) {
    String index;
    bool foundSurface = false;
    for (auto &s: into.parameters->getValue<ObjectConcept::surfacesProperty>()) {
        if (isASubConceptOfB(s.second.instanceId.s, "PouringSurface")) {
            index = s.first;
            foundSurface = true;
            break;
        }
    }
    if (!foundSurface) {
        cout << "Can not pour into " << into.instanceId.s << endl;
        return {};
    }
    Instance<ConceptList<PouringSurfaceConcept>> intoSurfaceInstance = mapGet(into.parameters->getValue<ObjectConcept::surfacesProperty>().m, index);
    cout << "IntoSurfaceInstance " << intoSurfaceInstance.instanceId.s << ": " << intoSurfaceInstance.geometry.get() << endl;
    ObjectSurface *intoSurface = dynamic_pointer_cast<ObjectSurface>(intoSurfaceInstance.geometry).get();
    auto const intoSurfaceInstancePoseVal = GetInstancePose::eval(intoSurfaceInstance);
    cout << "Into Surface Instance pose: " << intoSurfaceInstancePoseVal->q.toString() << endl;
    cout << "Into Surface pose: " << intoSurface->getPose().toString() << endl;

    return intoSurfaceInstance;

}


Instance<ConceptList<PourerSurfaceConcept>> DeterminePourerSurface(Instance<ConceptList<PourerConcept>> const &from) {
    String index;
    bool foundSurface = false;
    for (auto &s: from.parameters->getValue<ObjectConcept::surfacesProperty>()) {
        if (isASubConceptOfB(s.second.instanceId.s, "PouringSurface")) {
            index = s.first;
            foundSurface = true;
        }
    }
    if (!foundSurface) {
        cout << "Can not pour from " << from.instanceId.s << endl;
        return {};
    }
    Instance<ConceptList<PourerSurfaceConcept>> fromSurfaceInstance = mapGet(from.parameters->getValue<ObjectConcept::surfacesProperty>().m, index);
    cout << "FromSurfaceInstance " << fromSurfaceInstance.instanceId.s << ": " << fromSurfaceInstance.geometry.get()
         << endl;
    ObjectSurface *fromSurface = dynamic_pointer_cast<ObjectSurface>(fromSurfaceInstance.geometry).get();
    auto const fromSurfaceInstancePoseVal = GetInstancePose::eval(fromSurfaceInstance);
    cout << "From Surface Instance pose: " << fromSurfaceInstancePoseVal->q.toString() << endl;
    cout << "From Surface pose: " << fromSurface->getPose().toString() << endl;

    return fromSurfaceInstance;


}

Sequence<Location> DeterminePrePourGoalLocations(Instance<ConceptList<PourerConcept>> const &from, Instance<ConceptList<ContainerConcept>> const &into, Instance<ConceptList<PourerSurfaceConcept>> fromSurfaceInstance, Instance<ConceptList<PouringSurfaceConcept>> intoSurfaceInstance, Number angle, Number amount, Number pourdirection, Number height, Number time, Number spinAngle, ConceptLibrary::Pose const &endEffectorToObjectOrigin) {


    ObjectSurface *fromSurface = dynamic_pointer_cast<ObjectSurface>(fromSurfaceInstance.geometry).get();
    ObjectSurface *intoSurface = dynamic_pointer_cast<ObjectSurface>(intoSurfaceInstance.geometry).get();

    double const &pourAngle = angle.n;
    double const &pourAmount = amount.n;
    double const &pourDirection = pourdirection.n;
    double const &pourHeight = height.n;
    double const &pourTime = time.n;
    double pourSpinAngle = spinAngle.n;

    cout << "Pour angle = " << pourAngle << endl;
    cout << "Pour amount = " << pourAmount << endl;
    cout << "Pour direction = " << pourDirection << endl;
    cout << "Pour height = " << pourHeight << endl;
    cout << "Pour spin angle = " << pourSpinAngle << endl;
    cout << "Pour time = " << pourTime << endl;

    if (AndreiUtils::equal<double>(pourAmount, 0)) {
        cout << "Pour amount is 0! Don't execute pouring at all :)" << endl;
        return{};
    }

    // constants definition: F = from = object from which it is poured, I = into = object into which it is poured
    AndreiUtils::Posed const &I_q_IS = intoSurface->getSurfaceOrigPose();
    AndreiUtils::Posed const &F_q_FS = fromSurface->getSurfaceOrigPose();
    auto const W_q_I = GetInstancePose::eval(into);
    auto const W_q_F = GetInstancePose::eval(from);
    Eigen::Matrix3d I_r_IS = I_q_IS.getRotationAsMatrix();
    Eigen::Matrix3d F_r_FS = F_q_FS.getRotationAsMatrix();
    AndreiUtils::Posed const FS_q_F = F_q_FS.inverse();
    Eigen::Vector3d I_z_IS = I_r_IS * zAxis3d<double>();
    Eigen::Vector3d F_z_FS = F_r_FS * zAxis3d<double>();

    AndreiUtils::Posed goalPose1,goalPose2, goalPose3;
    Vector3d direction = Vector3d(cos(pourDirection), sin(pourDirection), 0);
    double l = into.parameters->getValue<ObjectConcept::interactionVolumeProperty>().n;  // bowl interaction volume
    cout << "Into interaction volume = " << l << endl;
    Eigen::Vector3d I_pourDirection = I_q_IS.getRotationAsMatrix() * direction;
    Vector3d I_p_F = I_pourDirection * l * 1.25;

    // bring "from" into interaction volume of "into" along the pouring direction
    AndreiUtils::Posed qTransform1(qIdentity<double>(), F_q_FS.getTranslation().z() * I_z_IS);
    AndreiUtils::Posed qTransform2(Eigen::Quaterniond(Eigen::AngleAxisd(
                                           -AndreiUtils::vectorAngle(AndreiUtils::zAxis3d<double>(), F_z_FS), yAxis3d<double>())),
                                   Eigen::Vector3d::Zero());
    AndreiUtils::Posed qTransform = qTransform1 * qTransform2;
    Eigen::Vector3d I_pourRotationAxis = I_z_IS.cross(I_pourDirection);
    Eigen::Matrix3d I_r0_FS = AndreiUtils::getOrientationFromAxes(I_pourDirection, I_pourRotationAxis, I_z_IS);
    goalPose1 = W_q_I->q * AndreiUtils::Posed(I_r0_FS, I_p_F) * qTransform * FS_q_F;
    auto goalPose1offset= goalPose1*endEffectorToObjectOrigin.q;



    // raise "from" above "into"
    Eigen::Vector3d pTmp1_FS = I_p_F + (pourHeight + l / 2) * I_z_IS;
    AndreiUtils::Posed I_q_FS_beforeGoingAbovePourPoint = AndreiUtils::Posed(I_r0_FS, pTmp1_FS) * qTransform;
    goalPose2 = W_q_I->q * I_q_FS_beforeGoingAbovePourPoint * FS_q_F;
    auto goalPose2offset= goalPose2*endEffectorToObjectOrigin.q;


    // determine pour point
    double minAxisSizeHalf = 0;
    for (long i = 0; i < intoSurface->getSurfacePointCount() - 1; ++i) {
        double axisSize = intoSurface->getSurfaceMatrixOrig().col(i).norm();
        cout << "axis size at axis " << i << " = " << axisSize << endl;
        if (i == 0 || axisSize / 2 < minAxisSizeHalf) {
            minAxisSizeHalf = axisSize / 2;
        }
    }
    cout << "MinAxisSize = " << minAxisSizeHalf << endl;
    Eigen::Vector3d I_pourPoint = I_q_IS.getTranslation() + I_pourDirection * minAxisSizeHalf * 0.75;
    cout << "Pour point " << I_pourPoint.transpose() << " vs. surface origin " << I_q_IS.getTranslation().transpose()
         << endl;

    //*
    // go to point above "into"
    Eigen::Vector3d pTmp2_FS = I_pourPoint + (pourHeight + l / 2) * I_z_IS;
    AndreiUtils::Posed I_q_FS_beforePour = AndreiUtils::Posed(I_r0_FS, pTmp2_FS) * qTransform;
    goalPose3 = W_q_I->q * I_q_FS_beforePour * FS_q_F;
    auto goalPose3offset= goalPose3*endEffectorToObjectOrigin.q;

    //*/

    // final goal pose of the pourer-surface coordinate frame
    // angle axis (I^z_IS x direction)
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(-pourAngle, I_pourRotationAxis));
    Eigen::Vector3d zFS = qRotate(rotation, I_z_IS);
    // the rotated vector below can be any vector in the x-y plane... must not necessarily be 'I_pourDirection'
    Eigen::Vector3d xFS = qRotate(rotation, I_pourDirection);
    xFS = qRotate(Eigen::Quaterniond(Eigen::AngleAxisd(pourSpinAngle, zFS)), xFS);
    Eigen::Matrix3d I_r_FS = AndreiUtils::getOrientationFromTwoAxes(zFS, xFS);
    Eigen::Vector3d pFinal_FS = I_pourPoint + pourHeight * I_z_IS;
    AndreiUtils::Posed I_q_FS_duringPour(I_r_FS, pFinal_FS);

    AndreiUtils::Posed finalGoalPose = W_q_I->q * I_q_FS_duringPour * FS_q_F;

    vector<Location> goalPointsVector;
    goalPointsVector.push_back(Location(goalPose1offset));
    goalPointsVector.push_back(Location(goalPose2offset));
    goalPointsVector.push_back(Location(goalPose3offset));
    Sequence<Location> prePourGoalLocations = Sequence(goalPointsVector);

    return prePourGoalLocations;

}

Sequence<Location> DeterminePourPath(Instance<ConceptList<PourerConcept>> const &from, Instance<ConceptList<ContainerConcept>> const &into, Instance<ConceptList<PourerSurfaceConcept>> fromSurfaceInstance, Instance<ConceptList<PouringSurfaceConcept>> intoSurfaceInstance, Number angle, Number amount, Number pourdirection, Number height, Number time, Number spinAngle, ConceptLibrary::Pose const &endEffectorToObjectOrigin) {


    ObjectSurface *fromSurface = dynamic_pointer_cast<ObjectSurface>(fromSurfaceInstance.geometry).get();
    ObjectSurface *intoSurface = dynamic_pointer_cast<ObjectSurface>(intoSurfaceInstance.geometry).get();

    double const &pourAngle = angle.n;
    double const &pourAmount = amount.n;
    double const &pourDirection = pourdirection.n;
    double const &pourHeight = height.n;
    double const &pourTime = time.n;
    double pourSpinAngle = spinAngle.n;

    cout << "Pour angle = " << pourAngle << endl;
    cout << "Pour amount = " << pourAmount << endl;
    cout << "Pour direction = " << pourDirection << endl;
    cout << "Pour height = " << pourHeight << endl;
    cout << "Pour spin angle = " << pourSpinAngle << endl;
    cout << "Pour time = " << pourTime << endl;

    if (AndreiUtils::equal<double>(pourAmount, 0)) {
        cout << "Pour amount is 0! Don't execute pouring at all :)" << endl;
        return {};
    }

    // constants definition: F = from = object from which it is poured, I = into = object into which it is poured
    AndreiUtils::Posed const &I_q_IS = intoSurface->getSurfaceOrigPose();
    AndreiUtils::Posed const &F_q_FS = fromSurface->getSurfaceOrigPose();
    auto const W_q_I = GetInstancePose::eval(into);
    auto const W_q_F = GetInstancePose::eval(from);
    Eigen::Matrix3d I_r_IS = I_q_IS.getRotationAsMatrix();
    Eigen::Matrix3d F_r_FS = F_q_FS.getRotationAsMatrix();
    AndreiUtils::Posed const FS_q_F = F_q_FS.inverse();
    Eigen::Vector3d I_z_IS = I_r_IS * zAxis3d<double>();
    Eigen::Vector3d F_z_FS = F_r_FS * zAxis3d<double>();

    AndreiUtils::Posed goalPose1, goalPose2, goalPose3;
    Vector3d direction = Vector3d(cos(pourDirection), sin(pourDirection), 0);
    double l = into.parameters->getValue<ObjectConcept::interactionVolumeProperty>().n;  // bowl interaction volume
    cout << "Into interaction volume = " << l << endl;
    Eigen::Vector3d I_pourDirection = I_q_IS.getRotationAsMatrix() * direction;
    Vector3d I_p_F = I_pourDirection * l * 1.25;

    // bring "from" into interaction volume of "into" along the pouring direction
    AndreiUtils::Posed qTransform1(qIdentity<double>(), F_q_FS.getTranslation().z() * I_z_IS);
    AndreiUtils::Posed qTransform2(Eigen::Quaterniond(Eigen::AngleAxisd(
                                           -AndreiUtils::vectorAngle(AndreiUtils::zAxis3d<double>(), F_z_FS), yAxis3d<double>())),
                                   Eigen::Vector3d::Zero());
    AndreiUtils::Posed qTransform = qTransform1 * qTransform2;
    Eigen::Vector3d I_pourRotationAxis = I_z_IS.cross(I_pourDirection);
    Eigen::Matrix3d I_r0_FS = AndreiUtils::getOrientationFromAxes(I_pourDirection, I_pourRotationAxis, I_z_IS);
    goalPose1 = W_q_I->q * AndreiUtils::Posed(I_r0_FS, I_p_F) * qTransform * FS_q_F;
    auto goalPose1offset = goalPose1 * endEffectorToObjectOrigin.q;



    // raise "from" above "into"
    Eigen::Vector3d pTmp1_FS = I_p_F + (pourHeight + l / 2) * I_z_IS;
    AndreiUtils::Posed I_q_FS_beforeGoingAbovePourPoint = AndreiUtils::Posed(I_r0_FS, pTmp1_FS) * qTransform;
    goalPose2 = W_q_I->q * I_q_FS_beforeGoingAbovePourPoint * FS_q_F;
    auto goalPose2offset = goalPose2 * endEffectorToObjectOrigin.q;


    // determine pour point
    double minAxisSizeHalf = 0;
    for (long i = 0; i < intoSurface->getSurfacePointCount() - 1; ++i) {
        double axisSize = intoSurface->getSurfaceMatrixOrig().col(i).norm();
        cout << "axis size at axis " << i << " = " << axisSize << endl;
        if (i == 0 || axisSize / 2 < minAxisSizeHalf) {
            minAxisSizeHalf = axisSize / 2;
        }
    }
    cout << "MinAxisSize = " << minAxisSizeHalf << endl;
    Eigen::Vector3d I_pourPoint = I_q_IS.getTranslation() + I_pourDirection * minAxisSizeHalf * 0.75;
    cout << "Pour point " << I_pourPoint.transpose() << " vs. surface origin " << I_q_IS.getTranslation().transpose()
         << endl;

    //*
    // go to point above "into"
    Eigen::Vector3d pTmp2_FS = I_pourPoint + (pourHeight + l / 2) * I_z_IS;
    AndreiUtils::Posed I_q_FS_beforePour = AndreiUtils::Posed(I_r0_FS, pTmp2_FS) * qTransform;
    goalPose3 = W_q_I->q * I_q_FS_beforePour * FS_q_F;
    auto goalPose3offset = goalPose3 * endEffectorToObjectOrigin.q;

    //*/

    // final goal pose of the pourer-surface coordinate frame
    // angle axis (I^z_IS x direction)
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(-pourAngle, I_pourRotationAxis));
    Eigen::Vector3d zFS = qRotate(rotation, I_z_IS);
    // the rotated vector below can be any vector in the x-y plane... must not necessarily be 'I_pourDirection'
    Eigen::Vector3d xFS = qRotate(rotation, I_pourDirection);
    xFS = qRotate(Eigen::Quaterniond(Eigen::AngleAxisd(pourSpinAngle, zFS)), xFS);
    Eigen::Matrix3d I_r_FS = AndreiUtils::getOrientationFromTwoAxes(zFS, xFS);
    Eigen::Vector3d pFinal_FS = I_pourPoint + pourHeight * I_z_IS;
    AndreiUtils::Posed I_q_FS_duringPour(I_r_FS, pFinal_FS);

    AndreiUtils::Posed finalGoalPose = W_q_I->q * I_q_FS_duringPour * FS_q_F;

    // keep I_z_IS-axis constant while lowering to pour!
    auto res = AndreiUtils::PoseDecoupledInterpolator<double>().compute(
            I_q_FS_beforePour, I_q_FS_duringPour, 6).getResult();

    res.erase(res.begin()); // deleting the first goal point since we are already there

    vector<Location> pourPathVector;

    res.erase(res.begin());

    for (size_t i = 0; i < res.size(); ++i) {
        auto& q = res[i];
        pourPathVector.push_back(Location(q));

    }

    Sequence<Location> pourPathSequence = Sequence(pourPathVector);


    return pourPathSequence;


}

Sequence<Location> CutAndReverse(const Sequence<Location>& inputSequence, Number indexToCutFrom) {
    // Copy elements [0, index) — exclusive of index

    auto inputVector = inputSequence.seq;
    size_t index= indexToCutFrom.n;
    std::vector<Location> result(inputVector.begin(), inputVector.begin() + min(index, inputVector.size()));

    // Reverse in place
    std::reverse(result.begin(), result.end());
    auto resultSequence = Sequence(result);

    return resultSequence;
}


Boolean isGraspSuitableForPour(Instance<ConceptList<ContainerConcept>> const &into, InstanceAccept<PourerSurfaceConcept> const &fromSurfaceInstance, InstanceAccept<PouringSurfaceConcept> const &intoSurfaceInstance, Number pourDirection_n, ConceptLibrary::Pose const &endEffectorToObjectOrigin) {




    auto const zTranslation = AndreiUtils::Posed {AndreiUtils::qIdentity<double>(), Eigen::Vector3d{0.0, 0.0, -0.1}};




    ObjectSurface *intoSurface = dynamic_pointer_cast<ObjectSurface>(intoSurfaceInstance.geometry).get();
    ObjectSurface *fromSurface = dynamic_pointer_cast<ObjectSurface>(fromSurfaceInstance.geometry).get();

    AndreiUtils::Posed const &I_q_IS = intoSurface->getSurfaceOrigPose();
    AndreiUtils::Posed const &F_q_FS = fromSurface->getSurfaceOrigPose();
    auto const W_q_I = GetInstancePose::eval(into);
    //auto const W_q_F = GetInstancePose::eval(from);
    Eigen::Matrix3d I_r_IS = I_q_IS.getRotationAsMatrix();
    Eigen::Matrix3d F_r_FS = F_q_FS.getRotationAsMatrix();
    AndreiUtils::Posed const FS_q_F = F_q_FS.inverse();
    Eigen::Vector3d I_z_IS = I_r_IS * zAxis3d<double>();
    Eigen::Vector3d F_z_FS = F_r_FS * zAxis3d<double>();


    double const pourDirection = pourDirection_n.n;
    Vector3d direction = Vector3d(cos(pourDirection), sin(pourDirection), 0);
    double l = into.parameters->getValue<ObjectConcept::interactionVolumeProperty>().n;  // bowl interaction volume
    Eigen::Vector3d I_pourDirection = I_q_IS.getRotationAsMatrix() * direction;
    Vector3d I_p_F = I_pourDirection * l * 1.25;

    // bring "from" into interaction volume of "into" along the pouring direction
    AndreiUtils::Posed qTransform1(qIdentity<double>(), F_q_FS.getTranslation().z() * I_z_IS);
    AndreiUtils::Posed qTransform2(Eigen::Quaterniond(Eigen::AngleAxisd(
                                           -AndreiUtils::vectorAngle(AndreiUtils::zAxis3d<double>(), F_z_FS), yAxis3d<double>())),
                                   Eigen::Vector3d::Zero());
    AndreiUtils::Posed qTransform = qTransform1 * qTransform2;
    Eigen::Vector3d I_pourRotationAxis = I_z_IS.cross(I_pourDirection);  // this is our vector
    Eigen::Matrix3d I_r0_FS = AndreiUtils::getOrientationFromAxes(I_pourDirection, I_pourRotationAxis, I_z_IS);
    AndreiUtils::Posed fromObjectPosePrePourGlobal = W_q_I->q * AndreiUtils::Posed(I_r0_FS, I_p_F) * qTransform * FS_q_F;

    auto const currentGraspEEPoseGlobal= fromObjectPosePrePourGlobal * endEffectorToObjectOrigin.q;
    auto const backTranslatedCurrentGraspEEPoseGlobal = currentGraspEEPoseGlobal * zTranslation;
    auto const intoSurfaceInstanceGlobalPose= GetInstancePose::eval(intoSurfaceInstance);
    auto const backTranslatedCurrentGraspEEPoseWrtIntoSurface = intoSurfaceInstanceGlobalPose->q.inverse() * backTranslatedCurrentGraspEEPoseGlobal;
    auto const fromObjectPosePrePourWrtIntoSurface = intoSurfaceInstanceGlobalPose->q.inverse() * fromObjectPosePrePourGlobal;
    auto const fromSurfacePosePrePourWrtIntoSurface = fromObjectPosePrePourWrtIntoSurface* I_q_IS;                                // this is our point
    auto const fromSurfaceXYZPrePourWrtIntoSurface =fromSurfacePosePrePourWrtIntoSurface.getTranslation();

    Eigen::Vector2d dir_xy(I_pourRotationAxis.x(), I_pourRotationAxis.y());
    Eigen::Vector2d P_xy(fromSurfaceXYZPrePourWrtIntoSurface.x(), fromSurfaceXYZPrePourWrtIntoSurface.y());
    dir_xy.normalize();

    double dx = dir_xy.x();
    double dy = dir_xy.y();
    double a = -dy;
    double b =  dx;
    double c = -(a * P_xy.x() + b * P_xy.y());

    double graspX = backTranslatedCurrentGraspEEPoseWrtIntoSurface.getTranslation().x();
    double graspY = backTranslatedCurrentGraspEEPoseWrtIntoSurface.getTranslation().y();

    if (c>0){
        if (a*graspX + b* graspY +c > 0){
            cout<<" !!!!Current grasp pose is not suitable for pour !"<< endl;
            return falseBoolean;
        }

        else{
            return trueBoolean;
        }
    }

    else {
        if (a*graspX + b* graspY +c < 0){
            cout<<" !!!!Current grasp pose is not suitable for pour !"<< endl;
            return falseBoolean;
        }
        else{
            return trueBoolean;
        }

    }


}

Location DetermineSafePoseForGraspSelection(Instance<ConceptList<ObjectConcept>> const &objectToBeGrasped){

    auto const objectPose = (GetInstancePose::eval(objectToBeGrasped));
    auto const objectPosedGlobal = objectPose.get().q;

    auto const pourRelatedSurfaceVal = ConceptLibrary::DeterminePouringSurface::eval(InstanceAccept<ContainerConcept>{objectToBeGrasped});
    auto const pourRelatedSurface = pourRelatedSurfaceVal.get();

    auto pourRelatedSurfacePose = (GetInstancePose::eval(pourRelatedSurface));
    auto const pourRelatedSurfacePosedGlobal =pourRelatedSurfacePose->q;

    auto const heightDifference = abs(pourRelatedSurfacePosedGlobal.getTranslation().z()-objectPosedGlobal.getTranslation().z())*1.4;
    auto const offset= AndreiUtils::Posed {Eigen::Quaterniond{1,0,0,0}, Eigen::Vector3d{0, 0, heightDifference}};
    auto const offsetSafePose = objectPosedGlobal*offset;
    auto const twist = AndreiUtils::Posed{AndreiUtils::qxRotation(M_PI), Eigen::Vector3d{0, 0, 0}};
    auto const finalSafePose = offsetSafePose* twist;
    auto const finalSafeLocation = Location(finalSafePose);

    return finalSafeLocation;

}
