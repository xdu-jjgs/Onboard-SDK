#define UTIL_OFFSETOF(type, member) ((size_t) & ((type *)0 )-> member)

typedef struct Box {
  int x;
  int y;
  int w;
  int h;
} Box;

typedef struct DetResultStruct {
  std::vector<Box> boxes;
} DetResultStruct;

