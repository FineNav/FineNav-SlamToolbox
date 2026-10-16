# SPDX-License-Identifier: MPL-2.0
# Copyright (c) FineNav
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Backend selection for optional C++ algorithm implementations."""

from .backend import NativeBackendUnavailable, native_api_version, native_available, resolve_backend

__all__ = [
    "NativeBackendUnavailable",
    "native_api_version",
    "native_available",
    "resolve_backend",
]
