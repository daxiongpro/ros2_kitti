Install env
1. Install the Python dependencies.
    pip install -r requirements
2. Install the pointnet2_3DSSD libarary.
    cd pcdet/ops/pointnet2/pointnet2_3DSSD/
    python setup.py develop
3. Install the pcdet library.
    cd ../../../../
    python setup.py develop
Use
    cd tools
    python detection_tracking.py

