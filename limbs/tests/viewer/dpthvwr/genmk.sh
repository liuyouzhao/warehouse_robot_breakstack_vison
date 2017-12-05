mkdir -p ./.mkst
cd ./.mkst
git clone https://github.com/liuyouzhao/code_samples
rm -Rf ./code_samples/linux_makefile/samples
rm -Rf ./code_samples/linux_makefile/test.c
cp -R ./code_samples/linux_makefile/* ../
cd ..
rm -Rf ./.mkst
