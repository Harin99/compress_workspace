# compress_workspace

This workspace is for compressing and recording image topics, and for decompressing compressed topics again.
If you want to use this, Follow below process. 

### 1. Git clone this workspace 
```
git clone https://github.com/Harin99/compress_workspace.git
```

### 2. Give execute permission to Python script file
```
cd ~/compress_workspace/src
chmod +x scripts/image_compressors.py
chmod +x scripts/image_decompressors.py 
```

### 3. Build the workspace 
```
cd ~/compress_workspace
catkin_make
source devel/setup.bash
```
