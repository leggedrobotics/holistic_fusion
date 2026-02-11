//
// Created by auv on 12/18/25.
//

#include <gtsam/base/serialization.h>
#include "graph_msf/core/Serialization.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <gtsam/base/GenericValue.h> // GTSAM_VALUE_EXPORT
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <boost/optional/optional_io.hpp>

using namespace graph_msf;
using namespace gtsam;

BOOST_CLASS_EXPORT_GUID(noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(noiseModel::Gaussian, "gtsam_noiseModel_Gaussian")
BOOST_CLASS_EXPORT_GUID(noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic")
//BOOST_CLASS_EXPORT_GUID(SharedNoiseModel, "gtsam_SharedNoiseModel")
//BOOST_CLASS_EXPORT_GUID(SharedDiagonal, "gtsam_SharedDiagonal")
BOOST_CLASS_EXPORT_GUID(PreintegratedImuMeasurements, "gtsam_PreintegratedImuMeasurements")
BOOST_CLASS_EXPORT_GUID(PreintegrationCombinedParams, "gtsam_PreintegrationCombinedParams")
BOOST_CLASS_EXPORT_GUID(PreintegratedCombinedMeasurements, "gtsam_PreintegratedCombinedMeasurements")


BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Cauchy , "gtsam_noiseModel_mEstimator_Cauchy")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam_noiseModel_mEstimator_Huber")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust")

GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::imuBias::ConstantBias);

BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Point3>, "gtsam::ExpressionFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Pose3>, "gtsam::ExpressionFactor<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<Pose3>  , "gtsam::PriorFactorPose3");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<Vector3>  , "gtsam::PriorFactorVector3");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>  , "gtsam::PriorFactorImuConstantBias6");
BOOST_CLASS_EXPORT_GUID(gtsam::CombinedImuFactor  , "gtsam::CombinedImuFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>  , "gtsam::BetweenFactor<gtsam::Pose3>");


std::string graph_msf::serializeFactor(const gtsam::NonlinearFactor::shared_ptr& fac){
    // Try to dynamic cast the factor to a shared pointer of CombinedImuFactor
    auto imu =
            boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(fac);
    // Try to dynamic cast the factor to a shared pointer of CombinedImuFactor
    auto pointFac =
            boost::dynamic_pointer_cast<gtsam::ExpressionFactor<gtsam::Point3>>(fac);
    // Try to dynamic cast the factor to a shared pointer of CombinedImuFactor
    auto poseFac =
            boost::dynamic_pointer_cast<gtsam::ExpressionFactor<gtsam::Pose3>>(fac);
    auto poseprior =
            boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(fac);

    // Check if the cast was successful
    if (imu) {
        std::cout << "Found a CombinedImuFactor!" << std::endl;
        CombinedImuFactor ff;
        std::string serialized = serialize(*imu);
        //std::cout << serialized << std::endl << std::endl;
        try{
            gtsam::deserialize(serialized, ff);   //
        }
        catch (const std::exception& e) {
            std::cerr << "Deserialize error: " << e.what() << "\n";
            return "failed";
        }
        std::cout << "deserialized" << std::endl << std::endl;
        assert_equal(*imu, ff);
        return serialized;
    } else if(pointFac) {
        std::cout << "Found a expression point3!" << std::endl;
//        std::string serialized = serialize(pointFac);
//        std::cout << serialized << std::endl << std::endl;

    }
    else if (poseFac){
        std::cout << "Found a expression pose3!" << std::endl;
//        std::string serialized = serialize(poseFac);
//        std::cout << serialized << std::endl << std::endl;
    }
    else if (poseprior){
        std::cout << "Found a prior pose3!" << std::endl;
        std::string serialized = serialize(*poseprior);
        std::cout << serialized << std::endl << std::endl;
        gtsam::PriorFactor<Pose3> facde;
        deserialize(serialized, facde);
        std::cout << "deserialized" << std::endl << std::endl;
        return serialized;
    }
    else{
        std::cout << "Found a factor of another type." << std::endl;
//        std::string serialized = serialize(fac);
//        std::cout << serialized << std::endl << std::endl;
    }
//    std::string serialized = serialize(fac);
//    std::cout << serialized << std::endl << std::endl;
    return "serialized";
}

void graph_msf::deserializeFactor(const std::string& serialized, const gtsam::NonlinearFactor::shared_ptr& fac){
    deserialize(serialized, *fac);
    return;
}
graph_msf::HFSerializedGraph graph_msf::serializeHFGraph(const gtsam::NonlinearFactorGraph& graph) {
    //Extract Core serializable factor graph
    gtsam::NonlinearFactorGraph core;
    std::vector<ExprFactorRecord> exprRecords;
    std::vector<int> coreFacInds, exprFacInds;
    for (size_t i = 0; i < graph.size(); i++) {
        auto& factor = graph[i];

        // Try to dynamic cast the factor to a shared pointer of CombinedImuFactor
        auto imu = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);

        if (imu){
            //std::cout<<"Found IMU factor"<<std::endl;
            core.push_back(imu);
            coreFacInds.push_back(i);
        }
        else{
            auto biasPrior = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(factor);
            auto posePrior = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor);
            auto velPrior = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Vector3>>(factor);
            auto btwFac = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
            if(biasPrior){
               // std::cout<<"Found Bias Prior factor"<<std::endl;
                core.push_back(biasPrior);
                coreFacInds.push_back(i);
            }
            else if(posePrior){
               // std::cout<<"Found Pose Prior factor"<<std::endl;
                core.push_back(posePrior);
                coreFacInds.push_back(i);
            }
            else if(velPrior){
                //std::cout<<"Found Velocity factor"<<std::endl;
                core.push_back(velPrior);
                coreFacInds.push_back(i);
            }
            else if(btwFac){
                core.push_back(btwFac);
                coreFacInds.push_back(i);
            }
            else {
                gtsam::SharedNoiseModel model;
                ExprFactorRecord  rec;

                if (auto exprP3 = boost::dynamic_pointer_cast<
                        gtsam::ExpressionFactor<gtsam::Point3>>(factor)) {
                    //std::cout<<"Found Expression Point3 factor"<<std::endl;
                    gtsam::Point3 meas = exprP3->measured();
                    model = exprP3->noiseModel();
                    rec.meas = {meas.x(), meas.y(), meas.z()};
                    rec.noise = model;
                   // std::cout << "Meas: " << meas.x() << "," << meas.y() << "," << meas.z() << ". noise model: ";

                } else if (auto exprP3 = boost::dynamic_pointer_cast<
                        gtsam::ExpressionFactor<gtsam::Pose3>>(factor)) {
                   // std::cout<<"Found Expression Pose3 factor"<<std::endl;
                    gtsam::Pose3 meas = exprP3->measured();
                    model = exprP3->noiseModel();
                    // Prefer robust 7d representation:
                    const auto t = meas.translation();
                    const auto q = meas.rotation().toQuaternion(); // Rot3 -> Quaternion

                    rec.meas = {t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z()};
                    rec.noise = model;
                    Eigen::Matrix4d m;
                    m <<0.006338, -0.999980,  0.000668,  0.302523,
                    -0.999910, -0.006329,  0.011798, -0.042547,
                    -0.011794, -0.000742, -0.999930, -0.012841,
                    0.000000,  0.000000,  0.000000,  1.000000;
                    rec.T_I_sensor = gtsam::Pose3(m);
                   // std::cout << "Meas: " << meas.matrix() << ". noise model: ";
                }
                else{
                    std::cout<<"ERROR Unknown Factor"<< typeid(*factor).name()<<std::endl;
                    std::cout<<"expression factor keys : ";
                    for(auto& k : factor->keys())
                        std::cout<<gtsam::Symbol(k)<<",";
                    std::cout<<std::endl;
                    throw std::runtime_error("Unknown Factor ");
                }

//                if (auto iso = boost::dynamic_pointer_cast<gtsam::noiseModel::Isotropic>(model)) {
//                    // iso->sigma() exists in many builds
//                    std::cout<<"isotropic"<<std::endl;
//                } else if (auto diag = boost::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(model)) {
//                    // diag->sigmas()
//                    std::cout<<"Diagonal"<<std::endl;
//                } else if (auto gauss = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model)) {
//                    // gauss->R() or covariance depending on API
//                    std::cout<<"Gaussian"<<std::endl;
//                }
//                else if (auto rob = boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(model)) {
//                    // gauss->R() or covariance depending on API
//                    std::cout<<"Robust"<<std::endl;
//                }
                rec.facName = typeid(*factor).name();
                rec.keys = factor->keys();
//                std::cout<<"expression factor keys : ";
//                for(auto& k : factor->keys())
//                    std::cout<<gtsam::Symbol(k)<<",";
//                std::cout<<std::endl;
                exprRecords.push_back(std::move(rec));
                exprFacInds.push_back(i);
            }

        }

//        gtsam::KeyVector involvedKeys = factor->keys();
//        std::cout << "Factor involves variables with keys: ";
//        for (gtsam::Key key : involvedKeys) {
//            gtsam::Symbol sym(key);
//            std::cout << sym.chr() << sym.index()<<" ";
//        }
//        std::cout << std::endl;

    }

    //
    gtsam::KeyVector involvedKeys = core.keyVector();
    //std::cout << "Factor graph involves variables with keys: ";
    for (gtsam::Key key : involvedKeys) {
        gtsam::Symbol sym(key);
        std::cout << sym.chr() << sym.index()<<" ";
    }
    std::cout << std::endl;
    // --- serialize core graph ---
    HFSerializedGraph out;
    out.coreGraphBytes = serializeBinary<NonlinearFactorGraph>(core);
    out.coreFacInds = coreFacInds;

    {
        //Serialize expression record
        //std::ofstream ofs("expression.bin", std::ios::binary);
        std::ostringstream oss(std::ios::binary);
        boost::archive::binary_oarchive oa(oss);
        oa << exprRecords;
        out.exprRecordBytes = oss.str();
        out.exprFacInds = exprFacInds;
    }


    return out;
}

NonlinearFactor::shared_ptr graph_msf::buildExpressionFactorFromRecord(const ExprFactorRecord& rec){
    //We are assuming that e have only two expression factors
    //  //  A) ExpressionFactor<Point3> : absolute position prior on translation(Pose3 key)
    //  //  B) ExpressionFactor<Pose3>  : pose prior (or some Pose3-valued expression)
    if (rec.meas.size() == 3){
        if (rec.keys.size() != 1)
            throw std::runtime_error("Point3 expr factor expects 1 key, got " +
                                     std::to_string(rec.keys.size()));

        Key k = rec.keys[0];
//        std::cout<<gtsam::Symbol(k)<<std::endl;
        Point3 z(rec.meas[0], rec.meas[1], rec.meas[2]);

        // Example expression: translation(X)
        gtsam::Expression<gtsam::Point3> expr =  gtsam::translation(gtsam::Expression<gtsam::Pose3>(gtsam::Symbol(k)));

        return boost::make_shared<ExpressionFactor<Point3>>(rec.noise,z, expr);
    }

    if (rec.meas.size() == 7) {
        if (rec.keys.size() != 2)
            throw std::runtime_error("Pose3 expr factor expects 2 keys, got " +
                                     std::to_string(rec.keys.size()));

        Key r = rec.keys[0];
        Key x = rec.keys[1];
        gtsam::Pose3 T_I_sensor = rec.T_I_sensor;
//        std::cout<<gtsam::Symbol(r)<<std::endl;
        const double tx = rec.meas[0], ty = rec.meas[1], tz = rec.meas[2];
        const double qw = rec.meas[3], qx = rec.meas[4], qy = rec.meas[5], qz = rec.meas[6];

        Rot3 R = Rot3::Quaternion(qw, qx, qy, qz);
        Pose3 z(R, gtsam::Point3(tx, ty, tz));

        // Example expression: identity pose (i.e., prior on X itself)
        // If your Pose3 expression is different, change this line accordingly.
        gtsam::Expression<gtsam::Pose3> expr =  gtsam::Expression<gtsam::Pose3>(gtsam::Symbol(x));
        gtsam::Expression<gtsam::Pose3> exp_T_W_fixedFrame= gtsam::Expression<gtsam::Pose3>(gtsam::Symbol(r));
        expr = transformPoseTo(exp_T_W_fixedFrame, expr);
        expr = gtsam::Pose3_(expr, &gtsam::Pose3::transformPoseFrom, gtsam::Pose3_(T_I_sensor));

        return boost::make_shared<ExpressionFactor<Pose3>>(rec.noise,z, expr);
    }

    throw std::runtime_error("Unknown expr record meas size = " +
                             std::to_string(rec.meas.size()) +
                             " facName=" + rec.facName);
}

NonlinearFactorGraph::shared_ptr graph_msf::deserializeHFGraph(const HFSerializedGraph& serGraphconst) {
    NonlinearFactorGraph::shared_ptr result(new NonlinearFactorGraph());
    const std::string& serializedCoreGraph = serGraphconst.coreGraphBytes;
    const std::string& serializedExprFactors = serGraphconst.exprRecordBytes;
    //Deserialize core graph
    deserializeBinary<NonlinearFactorGraph>(serializedCoreGraph, *result);
    //std::cout<<"loaded core graph of size: "<<result->size()<<std::endl;

    //read, construct theexpression factors and add to the graph
    std::vector<ExprFactorRecord> exprRecords;
    {
        std::istringstream iss(serializedExprFactors, std::ios::binary);
        boost::archive::binary_iarchive ia(iss);
        ia >> exprRecords;
    }
   // std::cout << "Loaded " << exprRecords.size() << " expression records\n";
    // rebuild expression factors
    for (const auto& rec : exprRecords) {
//        std::cout << "Rebuilding expr factor=" << rec.facName
//                  << " keys=" << rec.keys.size()
//                  << " meas_dim=" << rec.meas.size() << "\n";

        auto f = buildExpressionFactorFromRecord(rec);
        result->push_back(f);
    }
    //std::cout<<"Final graph size: "<<result->size()<<std::endl;

    NonlinearFactorGraph::shared_ptr out(new NonlinearFactorGraph());
    out->resize(result->size());
    std::vector<int> coreFacInds = serGraphconst.coreFacInds;
    std::vector<int> exprFacInds = serGraphconst.exprFacInds;
    assert(result->size() == (coreFacInds.size()+exprFacInds.size()));
   // std::cout<<"Allocated graph size: "<<out->size()<<std::endl;
    for (size_t i = 0; i < coreFacInds.size() ; i++) {
        size_t idx = coreFacInds[i];
        (*out)[idx] = (*result)[i];                // preserves shared_ptr
    }
    for (size_t i = 0; i < exprFacInds.size() ; i++) {
        size_t idx = exprFacInds[i];
        (*out)[idx] = (*result)[i+coreFacInds.size()];                // preserves shared_ptr
    }

    return out;
}

void graph_msf::saveHFSerializedGraph(const graph_msf::HFSerializedGraph& blob,
                           const std::string& path) {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) throw std::runtime_error("Failed to open " + path);

    boost::archive::binary_oarchive oa(ofs);
    oa << blob;
}
void graph_msf::loadHFSerializedGraph(const std::string& path, graph_msf::HFSerializedGraph& out) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) throw std::runtime_error("Failed to open " + path);

    boost::archive::binary_iarchive ia(ifs);
    ia >> out;
}
void graph_msf::testExpressionSerialization(){
    Key x0 = Symbol('x', 0);
    Expression<Pose3> X(x0);
    Expression<Point3> t = translation(X);

    Point3 gps_meas(1.0, 2.0, 3.0);
    auto model = noiseModel::Isotropic::Sigma(3, 0.5);

    NonlinearFactor::shared_ptr f =
            boost::make_shared<ExpressionFactor<Point3>>(model, gps_meas, t);
    // 3) Provide values so we can evaluate error
    Values values;
    values.insert(x0, Pose3(Rot3::Identity(), Point3(0.0, 0.0, 0.0)));

    double err_before = f->error(values);
    std::cout << "Error before serialization: " << err_before << "\n";
    // 4) Serialize the factor (as base pointer)
    std::stringstream ss;
    {
        boost::archive::binary_oarchive oa(ss);

        // Optional but helpful for polymorphic pointer serialization:
        // (Especially if you see "unregistered class" at runtime.)
        oa.register_type<ExpressionFactor<Point3>>();

        oa << f;
    }
    std::cout << "after serialization: " << "\n";

    // 5) Deserialize back
    NonlinearFactor::shared_ptr f2;
    {
        boost::archive::binary_iarchive ia(ss);

        // Must match registrations used for saving when needed
        ia.register_type<ExpressionFactor<Point3>>();

        ia >> f2;
    }

    if (!f2) {
        std::cerr << "Deserialized factor is null!\n";
        return ;
    }

    double err_after = f2->error(values);
    std::cout << "Error after  serialization: " << err_after << "\n";

    // 6) Check they match
    if (std::abs(err_before - err_after) > 1e-12) {
        std::cerr << "Mismatch after round-trip!\n";
        return;
    }

    std::cout << "ExpressionFactor serialize/deserialize test passed.\n";

    return;
}