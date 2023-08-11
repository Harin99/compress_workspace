# compress_workspace

This workspace is for compressing RGB image topics, and for decompressing compressed topics again.
If you want to use this, Follow below process. 

### 1. Git clone this workspace 
```
git clone https://github.com/Harin99/compress_workspace.git
```
### 2. Build the workspace 
```
cd ~/compress_workspace
catkin_make
source ~/compress_workspace/devel/setup.bash
```

### 3. Give execute permission to Python script file
```
cd ~/compress_workspace/src/image_compression
chmod +x scripts/image_compressors.py
chmod +x scripts/image_decompressors.py 
```

### 4. Run 
```
# In case of compress,
rosrun image_compression image_compressors.py 
# In case of decompress, 
rosrun image_compression image_decompressors.py 
```

#### Code modification : Compression 
Change the original topic name and the topic name to be compressed.

#### Code modification : Decompression 
Change the input_topics to your own topic names. And also output topics.
