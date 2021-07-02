/**
    This file is C++ implement of fast 2d pose extractor with python inference.
    It build in setup.py file.
*/

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION   //remove a Warning in build process

//"Python.h"包括一些标准头文件： <stdio.h>，<string.h>，<errno.h>，和 <stdlib.h>.
//如果后一个头文件在您的系统上不存在，它将直接声明函数malloc(),free()和realloc().
#include <Python.h>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <opencv2/core/core.hpp>
#include "numpy/arrayobject.h"

#include "extract_poses.hpp"

#define PY_SSIZE_T_CLEAN
using namespace std;

// translate PyArrayObject to cv::Mat
static std::vector<cv::Mat> wrap_features_map(PyArrayObject* py_features_map){
    int channel_nums = static_cast<int>(PyArray_SHAPE(py_features_map)[0]);
    int h = static_cast<int>(PyArray_SHAPE(py_features_map)[1]);
    int w = static_cast<int>(PyArray_SHAPE(py_features_map)[2]);
    float* data = static_cast<float*>(PyArray_DATA(py_features_map));

    std::vector<cv::Mat> feature_maps(channel_nums);
    for (long c_id=0; c_id < channel_nums; ++c_id){
        feature_maps[c_id] = cv::Mat(h, w, CV_32FC1,
                                     data + c_id * PyArray_STRIDE(py_features_map, 0) / sizeof(float),
                                     PyArray_STRIDE(py_features_map, 1));
    }

    return feature_maps;
}

// function to do something
static PyObject* extract_poses(PyObject* self, PyObject* args){
    PyArrayObject* py_heatmaps;
    PyArrayObject* py_pafs;
    int ratio;

    // PyArg_ParseTuple（）检查参数类型并将其转换为C值
    if(!PyArg_ParseTuple(args, "OOi", &py_heatmaps, &py_pafs, &ratio)){
        throw std::runtime_error("passed non-numpy array as argument");
    }
    std::vector<cv::Mat> heatmaps = wrap_features_map(py_heatmaps);
    std::vector<cv::Mat> pafs = wrap_features_map(py_pafs);
//    std::cout << heatmaps.size() << std::endl;
//    std::cout << heatmaps[0].size() << std::endl;

    std::vector<human_pose_estimation::HumanPose> poses =
                human_pose_estimation::extract_poses(heatmaps, pafs, ratio);

    size_t num_persons = poses.size();
    size_t num_keypoints = 0;
    if(num_persons > 0){
        num_keypoints = poses[0].keypoints.size();
    }

    float* out_data = new float[num_persons * (num_keypoints * 3 + 1)];
    for(size_t person_id = 0; person_id < num_persons; person_id++){
        for(size_t kpt_id = 0; kpt_id < num_keypoints * 3; kpt_id += 3){
            out_data[person_id * (num_keypoints * 3 + 1) + kpt_id + 0] = poses[person_id].keypoints[kpt_id / 3].x;
            out_data[person_id * (num_keypoints * 3 + 1) + kpt_id + 1] = poses[person_id].keypoints[kpt_id / 3].y;
            out_data[person_id * (num_keypoints * 3 + 1) + kpt_id + 2] = poses[person_id].keypoints[kpt_id / 3].z;
        }
        out_data[person_id * (num_keypoints * 3 + 1) + num_keypoints * 3] = poses[person_id].scores;
    }
    npy_intp dims[] = {static_cast<npy_intp>(num_persons), static_cast<npy_intp>(num_keypoints * 3 + 1)};
    PyObject *pArray = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, reinterpret_cast<void*>(out_data));
    PyArray_ENABLEFLAGS(reinterpret_cast<PyArrayObject*>(pArray), NPY_ARRAY_OWNDATA);
    return Py_BuildValue("(N)", pArray);
}

// Method definition object for this extension, these arguments mean:
// ml_name: The name of the method
// ml_meth: Function pointer to the method implementation
// ml_flags: Flags indicating special features of this method, such as
//          accepting arguments, accepting keyword arguments, being a
//          class method, or being a static method of a class.
// ml_doc:  Contents of this method's docstring
PyMethodDef method_table[] = {
    {"extract_poses", static_cast<PyCFunction>(extract_poses), METH_VARARGS,
    "Extract 2d poses from heatmaps and pafs"},
    {NULL, NULL, 0, NULL}
};

// Module definition
// The arguments of this structure tell Python what to call your extension,
// what it's methods are and where to look for it's method definitions
PyModuleDef pose_extractor_module = {
    PyModuleDef_HEAD_INIT,
    "pose_extractor",
    "Module for fast 2d pose extractor",
    -1,
    method_table
};

// Module initialization
// Python calls this function when importing your extension. It is important
// that this function is named PyInit_[[your_module_name]] exactly, and matches
// the name keyword argument in setup.py's setup() call.
PyMODINIT_FUNC PyInit_pose_extractor(void){
    PyObject* module = PyModule_Create(&pose_extractor_module);
    if(module == nullptr){
        return nullptr;
    }

    import_array()
    if(PyErr_Occurred()){
        return nullptr;
    }

    return module;
}

