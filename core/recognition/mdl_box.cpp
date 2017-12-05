#include "mdl_box.h"
#include "mm.h"
#include <vector>

std::vector< face_template_t* > s_tmplts;

static FILE *s_fd;
static int readline(FILE *f, char *buffer, size_t len)
{
   char c;
   int i;
   memset(buffer, 0, len);
   for (i = 0; i < len; i++)
   {
      int c = fgetc(f);

      if (!feof(f))
      {
         if (c == '\r')
            buffer[i] = 0;
         else if (c == '\n')
         {
            buffer[i] = 0;
            return i+1;
         }
         else
            buffer[i] = c;
      }
      else
      {
         return -1;
      }
   }

   return -1;
}

face_entity_s *face_entity_create()
{
    face_entity_s *pt = (face_entity_s*) DEBUG_MALLOC(sizeof(face_entity_s));
    pt->__construct();
    for(int i = 0; i < 4; i ++)
    {
        pt->juncts[i] = new junction_t();
        pt->juncts[i]->jid = i;
        pt->juncts[i]->owner = pt;
    }
    return pt;
}

void face_entity_destroy(face_entity_s **p)
{
    for(int i = 0; i < 4; i ++)
    {
        delete (*p)->juncts[i];
    }
    DEBUG_FREE(*p);
    *p = NULL;
}

void prm_open_file(const char *filename)
{
    s_fd = fopen(filename, "a");
}

void prm_open_file_read(const char *filename)
{
    s_fd = fopen(filename, "r");
}

void prm_append(int from, int to, int junction)
{
    char buf[64] = {0};
    sprintf(buf, "%d[%d]->%d\n", from, junction, to);
    fwrite(buf, strlen(buf), 1, s_fd);
}

void prm_begin_append()
{
}

void prm_end_append()
{
    fwrite("\n", 1, 1, s_fd);
}

void prm_close_file()
{
    fclose(s_fd);
    s_fd = NULL;
}

static face_entity_t *parse_entity(std::vector<char*> lines)
{
    if(lines.size() == 0)
    {
        return NULL;
    }

    face_entity_t *all[s_tmplts.size()];
    memset(all, 0, sizeof(void*) * s_tmplts.size());

    for(int i = 0; i < lines.size(); i ++)
    {
        char *line = lines[i];
        int from, to, junction;
        sscanf(line, "%d[%d]->%d\n", &from, &junction, &to);
        printf("%d[%d]->%d\n", from, junction, to);

        if(all[from] == NULL)
        {
            face_entity_t *efrom = face_entity_create();
            all[from] = efrom;
        }
        if(all[to] == NULL)
        {
            face_entity_t *eto = face_entity_create();
            all[to] = eto;
        }

        all[from]->ptmplt = s_tmplts[from];
        all[to]->ptmplt = s_tmplts[to];

        all[from]->juncts[junction]->pointers.push_back(all[to]);
        all[to]->parent = all[from];
    }

    for(int i = 0; i < s_tmplts.size(); i ++)
    {
        if(all[i] == NULL)
        {
            return NULL;
        }
        if(all[i]->parent == NULL)
        {
            return all[i];
        }
    }
    return NULL;
}

face_entity_t *prm_getnext_entity()
{
    std::vector<char*> lines;
    face_entity_t *entity;

    while(1)
    {
        char *buffer = (char*)DEBUG_MALLOC(128);
        int n = readline(s_fd, buffer, 128);
        if(n <= 0)
        {
            DEBUG_FREE(buffer);
            return NULL;
        }
        else if(n == 1)
        {
            DEBUG_FREE(buffer);
            break;
        }
        else
        {
            lines.push_back(buffer);
        }
    }
    entity = parse_entity(lines);
    for(int i = 0; i < lines.size(); i ++)
    {
        DEBUG_FREE(lines[i]);
    }
    return entity;
}

int face_belong_junct(face_entity_t *e, face_entity_t *p)
{
    for(int i = 0; i < 4; i ++)
    {
        for(int j = 0; j < p->juncts[i]->pointers.size(); j ++)
        {
            if(p->juncts[i]->pointers[j] == e)
            {
                return i;
            }
        }
    }
    return -1;
}

face_entity_t *face_clone(face_entity_t *src)
{
    face_entity_t *entity = (face_entity_t*) DEBUG_MALLOC(sizeof(face_entity_t));
    memcpy(entity, src, sizeof(face_entity_t));
    for(int i = 0; i < 4; i ++)
    {
        entity->juncts[i] = new junction_t();
    }
    return entity;
}

void face_tree_copy(face_entity_t *f, face_entity_t **fn, face_entity_t **root)
{
    if(*root == NULL)
    {
        *root = face_clone(f);
        *fn = *root;
    }

    for(int i = 0; i < 4; i ++)
    {
        if(f->juncts[i] != NULL)
        {
            for(int j = 0; j < f->juncts[i]->pointers.size(); j ++)
            {
                (*fn)->juncts[i]->pointers.push_back(face_clone((face_entity_t*)(f->juncts[i]->pointers[j])));
            }
        }
    }
}

void face_tree_destroy(face_entity_t **root)
{
    for(int i = 0; i < 4; i ++)
    {
        if((*root)->juncts[i] != NULL)
        {
            for(int j = 0; j < (*root)->juncts[i]->pointers.size(); j ++)
            {
                face_entity_t *face = (face_entity_t*)((*root)->juncts[i]->pointers[j]);
                face_tree_destroy(&face);
            }
            (*root)->juncts[i]->pointers.clear();
        }
    }
    for(int i = 0; i < 4; i ++)
    {
        delete (*root)->juncts[i];
    }
    DEBUG_FREE(*root);
}

void face_tree_print(face_entity_t *root)
{
    printf("%d", root->ptmplt->id);
    for(int i = 0; i < 4; i ++)
    {
        if(root->juncts[i] != NULL)
        {
            printf("[%d]", i);
            for(int j = 0; j < root->juncts[i]->pointers.size(); j ++)
            {
                face_tree_print((face_entity_t*)(root->juncts[i]->pointers[j]));
            }
        }
        printf("\n");
    }
}

void prm_set_templates(std::vector< face_template_t* > tmplts)
{
    s_tmplts.clear();
    for(int i = 0; i < tmplts.size(); i ++)
    {
        s_tmplts.push_back(tmplts[i]);
    }
}
