/* 
 * File:   detectors_utils.cpp
 * Author: daniele
 * 
 * Created on 20 marzo 2015, 22.19
 */

#include "detectors_utils.h"
#include "Bold3DM2Detector.h"
#include "Bold3DM2MultiDetector.h"
#include "Bold3DR2Detector.h"
#include "Bold3DRDetector.h"
#include "BoldDetector.h"
#include "Bold3DXDetector.h"
#include "Bold3DMDetector.h"
#include "Bold3DR2MultiDetector.h"
#include "Bold3DExtractor.h"
#include "Bold3DDescriptorMultiBunch.h"
#include "DFunctionB3DV2.h"
#include "DFunctionB2D.h"
#include "HybridDetector.h"
#include "DFunctionFPFH.h"
#include "DFunctionB3D4H.h"
#include "DFunctionB3DZ.h"
#include "DFunctionB3DV2Multi.h"
#include "DFunctionB3DV1Multi.h"
#include "DFunctionB3DV1.h"
#include "DFunctionFPFH2.h"
#include "DFunctionPPF.h"
#include "SiftDetector.h"
#include "DFunctionBD.h"

namespace visy {
    namespace detectors {
        namespace utils {

            /**
             * 
             * @param detector_name
             * @param parameters
             * @return 
             */
            Detector *
            buildDetectorFromString(std::string detector_name, visy::Parameters* parameters, bool is_model_detector) {
                bool use_occlusion_edges = false;
                int nbin = parameters->getInt("nbin");
                float f_th = parameters->getFloat("f_th");
                if (parameters->getInt("occlusion") >= 1) {
                    use_occlusion_edges = true && !is_model_detector;
                }
                std::vector<float> sizes = visy::Parameters::parseFloatArray(parameters->getString("sizes"));

                visy::detectors::Detector * detector;

                if (detector_name == "BOLD") {
                    detector = new visy::detectors::BoldDetector(sizes);
                } else if (detector_name == "B3D_R_V1") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            use_occlusion_edges,
                            5.0f,
                            2.0f,
                            25.0f,
                            0.05f,
                            visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD,
                            5,
                            2.0f,
                            f_th);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DV1(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                }else if (detector_name == "B3D_R_Z") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            use_occlusion_edges,
                            5.0f,
                            2.0f,
                            25.0f,
                            0.05f,
                            visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD,
                            5,
                            2.0f,
                            f_th);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DZ(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_BD") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            use_occlusion_edges,
                            5.0f,
                            2.0f,
                            25.0f,
                            0.05f,
                            visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD,
                            5,
                            2.0f,
                            f_th);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionBD(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "SIFT") {
                    detector = new visy::detectors::SiftDetector();
                }

                return detector;
            }

        }
    }
}

