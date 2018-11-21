#include <context.h>
#include <application.h>
#include <listener.h>
#include <mesh.h>
#include <color.h>
#include <utilities.h>
#include <string>
#include <sstream>
#include <android/sensor.h>
#include <android/log.h>
#include <math.h>
#include <cmath>
#include <vector>

#define earthRadiusKm 6371.0

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "arrow_sample", __VA_ARGS__))

using namespace WayRay;

class CheckPointObject : public GeoObject {
public:
    enum CP_STATE {
        HIDDEN,
        INITED,
        SHOWN
    };
    CP_STATE state = HIDDEN;

    std::shared_ptr<Mesh> initMesh;
    std::array<float, 3> initRotation;
    std::array<float, 3> initScale;
    std::shared_ptr<Texture> initTexture;

    std::shared_ptr<Mesh> objMesh;
    std::array<float, 3> objRotation;
    std::array<float, 3> objScale;
    std::shared_ptr<Texture> objTexture;

    CheckPointObject(std::string name, double latitude, double longitude, double altitude,
                     std::shared_ptr<Mesh> initMesh,
                     const std::array<float, 3> &initScale, const std::array<float, 3> &initRotation,
                     std::shared_ptr<Mesh> objMesh,
                     const std::array<float, 3> &objScale, const std::array<float, 3> &objRotation) : GeoObject(name) {
        setPose(Pose(latitude, longitude, altitude));
        this->initMesh = initMesh;
        this->initScale = initScale;
        this->initRotation = initRotation;
        this->initTexture = std::make_shared<Color>(Color::Palette::Blue);
        this->objMesh = objMesh;
        this->objScale = objScale;
        this->objRotation = objRotation;
        this->objTexture = std::make_shared<Color>(Color::Palette::Green);
    }

    void setInitModel() {
        setMesh(initMesh);
        setRotation(initRotation);
        setScale(initScale);
        setTexture(initTexture);
    }

    void setObjectModel() {
        setMesh(objMesh);
        setRotation(objRotation);
        setScale(objScale);
        setTexture(objTexture);
    }
};

class PointsManager {
private:
    int currentPoint = 0;
    std::vector<std::shared_ptr<CheckPointObject> > checkPoints;

public:
    PointsManager() {
        std::shared_ptr<Mesh> arrowModel = ResourceHelper::loadMesh("arrow.obj");
        std::shared_ptr<Mesh> questionModel = ResourceHelper::loadMesh("question.obj");;
        checkPoints = {{
                                   std::make_shared<CheckPointObject>("firstCheckPoint",
                                                                      47.641429, -122.139804, 123,
                                                                      questionModel,
                                                                      std::array<float, 3> {{0.02, 0.02, 0.02}},
                                                                      std::array<float, 3> {{3.14, 3.14, 4.71}},
                                                                      arrowModel,
                                                                      std::array<float, 3> {{1.5, 1.5, 1.5}},
                                                                      std::array<float, 3> {{0, 0, 1.57}}),
                                   std::make_shared<CheckPointObject>("secondCheckPoint",
                                                                      47.641968, -122.139804, 123,
                                                                      questionModel,
                                                                      std::array<float, 3> {{0.02, 0.02, 0.02}},
                                                                      std::array<float, 3> {{3.14, 3.14, 3.14}},
                                                                      arrowModel,
                                                                      std::array<float, 3> {{1.5, 1.5, 1.5}},
                                                                      std::array<float, 3> {{4.71, 0, 0}}),
                                   std::make_shared<CheckPointObject>("thirdCheckPoint",
                                                                      47.642354, -122.139804, 123,
                                                                      questionModel,
                                                                      std::array<float, 3> {{0.02, 0.02, 0.02}},
                                                                      std::array<float, 3> {{3.14, 3.14, 3.14}},
                                                                      arrowModel,
                                                                      std::array<float, 3> {{1.5, 1.5, 1.5}},
                                                                      std::array<float, 3> {{0, 0, 0}}),
                                   std::make_shared<CheckPointObject>("fourthCheckPoint",
                                                                      47.642412, -122.139234, 123,
                                                                      arrowModel,
                                                                      std::array<float, 3> {{1.5, 1.5, 1.5}},
                                                                      std::array<float, 3> {{1.57, 0, 0}},
                                                                      arrowModel,
                                                                      std::array<float, 3> {{1.5, 1.5, 1.5}},
                                                                      std::array<float, 3> {{1.57, 0, 0}})
                       }};

    };

    ~PointsManager() {};

    std::shared_ptr<CheckPointObject> getCurrentPointMark() {
        return checkPoints[currentPoint];
    }

    void initNextPoint() {
        LOGI("Initializing next point");
        if (currentPoint == 3) {
            return;
        }
        hidePoint();
        currentPoint++;
        initPoint();
    }

    void initPoint() {
        LOGI("Initializing point %i", currentPoint);
        std::shared_ptr<CheckPointObject> mark = getCurrentPointMark();
        if (mark->state == CheckPointObject::CP_STATE::INITED) {
            return;
        }
        hidePoint();
        mark->setInitModel();
        Context::get()->getScene().add(mark);
        mark->state = CheckPointObject::CP_STATE::INITED;
        LOGI("Point %i initialized", currentPoint);
    }

    void showPoint() {
        LOGI("Showing point %i", currentPoint);
        std::shared_ptr<CheckPointObject> mark = getCurrentPointMark();
        if (mark->state == CheckPointObject::CP_STATE::SHOWN) {
            return;
        }
        hidePoint();
        mark->setObjectModel();
        Context::get()->getScene().add(mark);
        mark->state = CheckPointObject::CP_STATE::SHOWN;
        LOGI("Point %i shown", currentPoint);
    }

    void hidePoint() {
        LOGI("Removing point %i", currentPoint);
        std::shared_ptr<CheckPointObject> mark = getCurrentPointMark();
        Context::get()->getScene().remove(mark);
        mark->state = CheckPointObject::CP_STATE::HIDDEN;
        LOGI("Point %i removed", currentPoint);
    }
};

class PoseChangeListener : public Listener<Pose> {
public:
    PoseChangeListener(PointsManager* manager) : pointsManager(manager) { }

    void changed(std::shared_ptr<Pose> pose) {
        LOGI("POSITION: %f, %f, %f", pose->latitude, pose->longitude, pose->altitude);
        std::shared_ptr<CheckPointObject> mark = pointsManager->getCurrentPointMark();
        if (mark == nullptr) {
            return;
        }
        double dist = distanceEarth(
                mark->getPose().latitude, mark->getPose().longitude,
                pose->latitude, pose->longitude);
        LOGI("DISTANCE TO POINT: %f", dist);
        if (dist <= 0.01) {
            LOGI("INIT NEXT POINT VIEW");
            pointsManager->initNextPoint();
        } else if (dist <= 0.03) {
            LOGI("CHANGE POINT VIEW");
            pointsManager->showPoint();
        }
    }

    double deg2rad(double deg) {
        return (deg * M_PI / 180);
    }

    double rad2deg(double rad) {
        return (rad * 180 / M_PI);
    }

    double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
        double lat1r, lon1r, lat2r, lon2r, u, v;
        lat1r = deg2rad(lat1d);
        lon1r = deg2rad(lon1d);
        lat2r = deg2rad(lat2d);
        lon2r = deg2rad(lon2d);
        u = sin((lat2r - lat1r) / 2);
        v = sin((lon2r - lon1r) / 2);
        return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
    }

private:
    PointsManager* pointsManager;
};

class ArrowSampleARApp : public Application {

    PointsManager *pointsManager = nullptr;
    std::shared_ptr<PoseChangeListener> poseChangeListener;

    virtual void onStart() override {
        pointsManager = new PointsManager();

        LOGI("Registering position listener");
        poseChangeListener = std::make_shared<PoseChangeListener>(pointsManager);
        Context::get()->getVehicleState().registerPoseChangeListener(poseChangeListener);

        LOGI("Starting alpha test");
        pointsManager->initPoint();
        LOGI("Application started");
    }

    virtual void onStop() override {
        LOGI("Handling application stop");
        pointsManager->hidePoint();

        Context::get()->getVehicleState().unregisterPoseChangeListener(poseChangeListener);

        delete pointsManager;
    }

};

extern "C" {
Application* truear_app_create() {
    return new ArrowSampleARApp();
}
}
