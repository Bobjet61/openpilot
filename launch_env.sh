#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# b8x is a pinned validation branch. Updates require an intentional reinstall
# or an SSH session that explicitly clears DisableUpdates.
export DISABLE_AUTO_UPDATES=1

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="10.1"
fi

export STAGING_ROOT="/data/safe_staging"
