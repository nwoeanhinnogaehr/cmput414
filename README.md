# Team KANA - Mesh Compression Project

**Build Instructions:**

  > `cd /test`
  >
  > `./build.sh`

**Run Instructions:**

  > `cd /test`
  >
  > `./cmput414_bin [FILENAME].[off|obj|ply] [1|2|3|4|5|6|7|8]`
  >
    where
    1 = vector sum
    2 = normals
    3 = manhattan
    4 = euclidean
    5 = view weight
    6 = circulation angle sum
    7 = DFS angle sum
    8 = random

**Use Instructions.**

  > `Press 1 to decimate.`
  >
  > `Press 2 to reconstruct.`
  >
  > `Press 3 to auto-output several image files to our image folder (for later processing.)`
  >
  > `Press r to reset the model completely.`
  >
  > `Press l to toggle wiremesh.`
  >
  > `Press c to toggle cost function heatmapping.`
