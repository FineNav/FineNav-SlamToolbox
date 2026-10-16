// SPDX-License-Identifier: MPL-2.0
// Copyright (c) FineNav
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_local_hash_voxel(py::module_& module);
void bind_raycast_voxel(py::module_& module);

PYBIND11_MODULE(_native, module) {
    module.doc() = "Native compute backends for slam_toolbox";
    module.attr("api_version") = 5;
    bind_local_hash_voxel(module);
    bind_raycast_voxel(module);
}
