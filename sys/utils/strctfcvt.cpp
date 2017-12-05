#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

int save_strct(const char *file, void *ptr, int siz)
{
    FILE *f = fopen(file, "w+");
    if(f == 0 || ptr == 0)
    {
        return -1;
    }
    fwrite((char*)ptr, siz, 1, f);
    fclose(f);
    return 0;
}

int load_strct(const char *file, void *ptr, int siz)
{
    int rc = 0;
    FILE *f = fopen(file, "w+");
    if(f == 0 || ptr == 0)
    {
        return -1;
    }
    rc = fread((char*)ptr, siz, 1, f);

    if(rc <= 0)
    {
        return -1;
    }

    fclose(f);
    return 0;
}

#ifdef __cplusplus
}
#endif
