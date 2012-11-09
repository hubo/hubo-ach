rm bsd.src
rm tmp
rm tmp1
echo '*/' >> tmp
echo '/*' | cat - License >> tmp1
cat tmp1 | cat - tmp >> bsd.src
rm tmp
rm tmp1

cat bsd.src | cat - hubo-default.c >> tmp.c
mv tmp.c hubo-default.c

cat bsd.src | cat - hubo-esdcan.c >> tmp.c
mv tmp.c hubo-esdcan.c

cat bsd.src | cat - hubo-loop.c >> tmp.c
mv tmp.c hubo-loop.c

cat bsd.src | cat - hubo-main.c >> tmp.c
mv tmp.c hubo-main.c

cat bsd.src | cat - hubo-socketcan.c >> tmp.c
mv tmp.c hubo-socketcan.c


cat bsd.src | cat - hubo-console.cpp >> tmp.c
mv tmp.c hubo-console.cpp
