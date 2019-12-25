
## Your links in an array
echo "starting"


declare -a arr="https://github.com/cogniteam/cognitao.git"
#echo "remove old version"
#cd ..

## Folder to store each of these git repos
folder=$(pwd)
echo "folder----> $folder"


rm -rf "cognitao.git"
## Go through each link in array
for i in "${arr[@]}"
do
    ## Use basename to extract folder name from link
    git clone $i $folder/$(basename $i) 
    echo "finished"	
done


exit 0

### this part wiil be inside cmakelist node

