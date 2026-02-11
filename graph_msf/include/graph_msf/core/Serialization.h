//
// Created by auv on 12/18/25.
//

#ifndef HOLISTIC_WS_SERIALIZATION_H
#define HOLISTIC_WS_SERIALIZATION_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>


namespace graph_msf {

struct ExprFactorRecord{
    std::string facName;
    std::vector<double> meas;
    gtsam::KeyVector keys;
    gtsam::SharedNoiseModel noise;
    gtsam::Pose3 T_I_sensor;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & facName;
        ar & meas;
        ar & keys;
        ar & noise;
        ar & T_I_sensor;
    }
};

struct HFSerializedGraph {
    std::vector<int> coreFacInds;
    std::vector<int> exprFacInds;
    std::string coreGraphBytes;     // GTSAM serialized NonlinearFactorGraph
    std::string exprRecordBytes;    // Boost serialized ExprFactorRecord vector
    template <class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/) {
        ar & coreFacInds;
        ar & exprFacInds;
        ar & coreGraphBytes;
        ar & exprRecordBytes;
    }
};
gtsam::NonlinearFactor::shared_ptr  buildExpressionFactorFromRecord(const ExprFactorRecord& rec);
void testExpressionSerialization();
std::string serializeFactor(const gtsam::NonlinearFactor::shared_ptr& fac);
void deserializeFactor(const std::string& serialized, const gtsam::NonlinearFactor::shared_ptr& fac);
HFSerializedGraph serializeHFGraph(const gtsam::NonlinearFactorGraph& graph);
gtsam::NonlinearFactorGraph::shared_ptr deserializeHFGraph(const HFSerializedGraph& serGraph);
void saveHFSerializedGraph(const HFSerializedGraph& blob,const std::string& path);
void loadHFSerializedGraph(const std::string& path, HFSerializedGraph& out);
}

#endif //HOLISTIC_WS_SERIALIZATION_H
