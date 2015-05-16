for line in `find . -name 'obj*'`; do 
    pcl_pcd2ply $line $line.ply -format 0
done
