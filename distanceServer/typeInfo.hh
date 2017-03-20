
class t_TypedPtr {
public:
  t_TypedPtr() {};
  t_TypedPtr(int t, char *n, void *v) {
    type = t;
    name = n;
    value =v;
  };
  int type;
  char *name;
  void *value;  
};

typedef t_TypedPtr *t_AttribArray;

class t_Attrib
{
public:
  enum tinfo {
    TI_CHAR, TI_SHORT, TI_LONG, TI_FLOAT, TI_DOUBLE, TI_STRING, TI_END
  };
  t_TypedPtr *tp;
  t_Attrib(t_TypedPtr *tpi) {this->tp = tpi;};
  void receive(char **buf);
  void transmit(char **buf);
};

#define TI_END_ATTR t_TypedPtr(t_Attrib::TI_END, NULL, NULL)

