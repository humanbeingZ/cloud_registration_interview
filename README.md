## Cloud Registeration Using ICP Algorithm


To compile this example:
```
git clone git@github.com:humanbeingZ/cloud_registration_interview.git
cd cloud_registration_interview
mkdir build
cd build
cmake ..
make -j8
```

To run the example:
```
./cloud_registration ../data/cloud_002.pcd ../data/cloud_003.pcd
```

**Attention**:  
Since time is limited, this example does not align the given point clouds very well.  
To improve upon this, you can utilize normal information and use point-to-plane ICP.  
