/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.9
 * 
 * This file is not intended to be easily readable and contains a number of 
 * coding conventions designed to improve portability and efficiency. Do not make
 * changes to this file unless you know what you are doing--modify the SWIG 
 * interface file instead. 
 * ----------------------------------------------------------------------------- */

#ifndef SWIG_Leap_WRAP_H_
#define SWIG_Leap_WRAP_H_

#include <map>
#include <string>


class SwigDirector_Listener : public Leap::Listener, public Swig::Director {

public:
    SwigDirector_Listener(PyObject *self);
    virtual ~SwigDirector_Listener();
    virtual void onInit(Leap::Controller const &arg0);
    virtual void onConnect(Leap::Controller const &arg0);
    virtual void onDisconnect(Leap::Controller const &arg0);
    virtual void onExit(Leap::Controller const &arg0);
    virtual void onFrame(Leap::Controller const &arg0);
    virtual void onFocusGained(Leap::Controller const &arg0);
    virtual void onFocusLost(Leap::Controller const &arg0);


/* Internal Director utilities */
public:
    bool swig_get_inner(const char* swig_protected_method_name) const {
      std::map<std::string, bool>::const_iterator iv = swig_inner.find(swig_protected_method_name);
      return (iv != swig_inner.end() ? iv->second : false);
    }

    void swig_set_inner(const char* swig_protected_method_name, bool val) const
    { swig_inner[swig_protected_method_name] = val;}

private:
    mutable std::map<std::string, bool> swig_inner;


#if defined(SWIG_PYTHON_DIRECTOR_VTABLE)
/* VTable implementation */
    PyObject *swig_get_method(size_t method_index, const char *method_name) const {
      PyObject *method = vtable[method_index];
      if (!method) {
        swig::SwigVar_PyObject name = SWIG_Python_str_FromChar(method_name);
        method = PyObject_GetAttr(swig_get_self(), name);
        if (!method) {
          std::string msg = "Method in class Listener doesn't exist, undefined ";
          msg += method_name;
          Swig::DirectorMethodException::raise(msg.c_str());
        }
        vtable[method_index] = method;
      }
      return method;
    }
private:
    mutable swig::SwigVar_PyObject vtable[7];
#endif

};


#endif
