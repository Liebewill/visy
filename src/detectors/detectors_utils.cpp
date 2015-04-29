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
                if (parameters->getInt("occlusion") >= 1) {
                    use_occlusion_edges = true && !is_model_detector;
                }
                std::vector<float> sizes = visy::Parameters::parseFloatArray(parameters->getString("sizes"));

                visy::detectors::Detector * detector;

                if (detector_name == "BOLD3DM") {
                    detector = new visy::detectors::Bold3DMDetector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD3DM2") {
                    detector = new visy::detectors::Bold3DM2Detector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD3DM2MULTI") {
                    detector = new visy::detectors::Bold3DM2MultiDetector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD3DR2") {
                    detector = new visy::detectors::Bold3DR2Detector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD3DR2MULTI") {
                    detector = new visy::detectors::Bold3DR2MultiDetector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD3DR") {
                    detector = new visy::detectors::Bold3DRDetector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD3DX") {
                    detector = new visy::detectors::Bold3DXDetector(sizes, parameters->getInt("nbin"), !use_occlusion_edges);
                } else if (detector_name == "BOLD") {
                    detector = new visy::detectors::BoldDetector(sizes);
                } else if (detector_name == "B3D_R_V1") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DV1(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_V1_MULTI") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DV1Multi(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_3ANGLE") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DV2(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_3ANGLE_MULTI") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DV2Multi(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_4ANGLE") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3D4H(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_2ANGLE") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB2D(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_PPF") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionPPF(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_FPFH") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionFPFH(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_FPFH2") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionFPFH2(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_Z") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DZ(nbin), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                }  else if (detector_name == "B3D_R_Z_MULTI") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DZ(nbin,true), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                } else if (detector_name == "B3D_R_Z_MULTI") {
                    visy::extractors::Extractor* extractor = new visy::extractors::Bold3DExtractor(
                            !use_occlusion_edges, 5.0f, 2.0f, 25.0f, 0.001f, visy::tools::VISY_TOOLS_EDGEDETECTION_METHOD_BOLD_LSD);
                    visy::descriptors::Descriptor* descriptor = new visy::descriptors::Bold3DDescriptorMultiBunch(
                            nbin, sizes, new visy::descriptors::DFunctionB3DZ(nbin, true), visy::descriptors::Bold3DDescriptorMultiBunch::BUNCH_METHOD_RADIUS);
                    detector = new visy::detectors::HybridDetector(detector_name, extractor, descriptor);
                }

                return detector;
            }

        }
    }
}

