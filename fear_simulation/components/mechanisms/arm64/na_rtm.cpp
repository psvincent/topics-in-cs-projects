/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#define _pval pval
// clang-format off
#include "md1redef.h"
#include "section_fwd.hpp"
#include "nrniv_mf.h"
#include "md2redef.h"
#include "nrnconf.h"
// clang-format on
#include "neuron/cache/mechanism_range.hpp"
static constexpr auto number_of_datum_variables = 3;
static constexpr auto number_of_floating_point_variables = 18;
namespace {
template <typename T>
using _nrn_mechanism_std_vector = std::vector<T>;
using _nrn_model_sorted_token = neuron::model_sorted_token;
using _nrn_mechanism_cache_range = neuron::cache::MechanismRange<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_mechanism_cache_instance = neuron::cache::MechanismInstance<number_of_floating_point_variables, number_of_datum_variables>;
using _nrn_non_owning_id_without_container = neuron::container::non_owning_identifier_without_container;
template <typename T>
using _nrn_mechanism_field = neuron::mechanism::field<T>;
template <typename... Args>
void _nrn_mechanism_register_data_fields(Args&&... args) {
  neuron::mechanism::register_data_fields(std::forward<Args>(args)...);
}
}
 
#if !NRNGPU
#undef exp
#define exp hoc_Exp
#if NRN_ENABLE_ARCH_INDEP_EXP_POW
#undef pow
#define pow hoc_pow
#endif
#endif
 
#define nrn_init _nrn_init__na_rtm
#define _nrn_initial _nrn_initial__na_rtm
#define nrn_cur _nrn_cur__na_rtm
#define _nrn_current _nrn_current__na_rtm
#define nrn_jacob _nrn_jacob__na_rtm
#define nrn_state _nrn_state__na_rtm
#define _net_receive _net_receive__na_rtm 
#define rate rate__na_rtm 
#define states states__na_rtm 
 
#define _threadargscomma_ _ml, _iml, _ppvar, _thread, _globals, _nt,
#define _threadargsprotocomma_ Memb_list* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt,
#define _internalthreadargsprotocomma_ _nrn_mechanism_cache_range* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt,
#define _threadargs_ _ml, _iml, _ppvar, _thread, _globals, _nt
#define _threadargsproto_ Memb_list* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt
#define _internalthreadargsproto_ _nrn_mechanism_cache_range* _ml, size_t _iml, Datum* _ppvar, Datum* _thread, double* _globals, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *hoc_getarg(int);
 
#define t _nt->_t
#define dt _nt->_dt
#define gbar _ml->template fpfield<0>(_iml)
#define gbar_columnindex 0
#define minfvhalf _ml->template fpfield<1>(_iml)
#define minfvhalf_columnindex 1
#define minfk _ml->template fpfield<2>(_iml)
#define minfk_columnindex 2
#define hinfvhalf _ml->template fpfield<3>(_iml)
#define hinfvhalf_columnindex 3
#define hinfk _ml->template fpfield<4>(_iml)
#define hinfk_columnindex 4
#define ina _ml->template fpfield<5>(_iml)
#define ina_columnindex 5
#define minf _ml->template fpfield<6>(_iml)
#define minf_columnindex 6
#define hinf _ml->template fpfield<7>(_iml)
#define hinf_columnindex 7
#define mtau _ml->template fpfield<8>(_iml)
#define mtau_columnindex 8
#define htau _ml->template fpfield<9>(_iml)
#define htau_columnindex 9
#define g _ml->template fpfield<10>(_iml)
#define g_columnindex 10
#define m _ml->template fpfield<11>(_iml)
#define m_columnindex 11
#define h _ml->template fpfield<12>(_iml)
#define h_columnindex 12
#define ena _ml->template fpfield<13>(_iml)
#define ena_columnindex 13
#define Dm _ml->template fpfield<14>(_iml)
#define Dm_columnindex 14
#define Dh _ml->template fpfield<15>(_iml)
#define Dh_columnindex 15
#define v _ml->template fpfield<16>(_iml)
#define v_columnindex 16
#define _g _ml->template fpfield<17>(_iml)
#define _g_columnindex 17
#define _ion_ena *(_ml->dptr_field<0>(_iml))
#define _p_ion_ena static_cast<neuron::container::data_handle<double>>(_ppvar[0])
#define _ion_ina *(_ml->dptr_field<1>(_iml))
#define _p_ion_ina static_cast<neuron::container::data_handle<double>>(_ppvar[1])
#define _ion_dinadv *(_ml->dptr_field<2>(_iml))
 /* Thread safe. No static _ml, _iml or _ppvar. */
 static int hoc_nrnpointerindex =  -1;
 static _nrn_mechanism_std_vector<Datum> _extcall_thread;
 static Prop* _extcall_prop;
 /* _prop_id kind of shadows _extcall_prop to allow validity checking. */
 static _nrn_non_owning_id_without_container _prop_id{};
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_rate(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mechtype);
#endif
 static void _hoc_setdata();
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 {"setdata_na_rtm", _hoc_setdata},
 {"rate_na_rtm", _hoc_rate},
 {0, 0}
};
 
/* Direct Python call wrappers to density mechanism functions.*/
 static double _npy_rate(Prop*);
 
static NPyDirectMechFunc npy_direct_func_proc[] = {
 {"rate", _npy_rate},
 {0, 0}
};
 /* declare global and static user variables */
 #define gind 0
 #define _gth 0
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 {0, 0, 0}
};
 static HocParmUnits _hoc_parm_units[] = {
 {"gbar_na_rtm", "siemens/cm2"},
 {"ina_na_rtm", "mA/cm2"},
 {"mtau_na_rtm", "ms"},
 {"htau_na_rtm", "ms"},
 {"g_na_rtm", "siemens/cm2"},
 {0, 0}
};
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 {0, 0}
};
 static DoubVec hoc_vdoub[] = {
 {0, 0, 0}
};
 static double _sav_indep;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 _prop_id = _nrn_get_prop_id(_prop);
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 static void nrn_alloc(Prop*);
static void nrn_init(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_state(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 static void nrn_cur(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void nrn_jacob(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(Prop*, int, neuron::container::data_handle<double>*, neuron::container::data_handle<double>*, double*, int);
static void _ode_spec(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
static void _ode_matsol(_nrn_model_sorted_token const&, NrnThread*, Memb_list*, int);
 
#define _cvode_ieq _ppvar[3].literal_value<int>()
 static void _ode_matsol_instance1(_internalthreadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"na_rtm",
 "gbar_na_rtm",
 "minfvhalf_na_rtm",
 "minfk_na_rtm",
 "hinfvhalf_na_rtm",
 "hinfk_na_rtm",
 0,
 "ina_na_rtm",
 "minf_na_rtm",
 "hinf_na_rtm",
 "mtau_na_rtm",
 "htau_na_rtm",
 "g_na_rtm",
 0,
 "m_na_rtm",
 "h_na_rtm",
 0,
 0};
 static Symbol* _na_sym;
 
 /* Used by NrnProperty */
 static _nrn_mechanism_std_vector<double> _parm_default{
     0, /* gbar */
     41.03, /* minfvhalf */
     -7.1, /* minfk */
     45.32, /* hinfvhalf */
     4.04, /* hinfk */
 }; 
 
 
extern Prop* need_memb(Symbol*);
static void nrn_alloc(Prop* _prop) {
  Prop *prop_ion{};
  Datum *_ppvar{};
   _ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
    _nrn_mechanism_access_dparam(_prop) = _ppvar;
     _nrn_mechanism_cache_instance _ml_real{_prop};
    auto* const _ml = &_ml_real;
    size_t const _iml{};
    assert(_nrn_mechanism_get_num_vars(_prop) == 18);
 	/*initialize range parameters*/
 	gbar = _parm_default[0]; /* 0 */
 	minfvhalf = _parm_default[1]; /* 41.03 */
 	minfk = _parm_default[2]; /* -7.1 */
 	hinfvhalf = _parm_default[3]; /* 45.32 */
 	hinfk = _parm_default[4]; /* 4.04 */
 	 assert(_nrn_mechanism_get_num_vars(_prop) == 18);
 	_nrn_mechanism_access_dparam(_prop) = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0] = _nrn_mechanism_get_param_handle(prop_ion, 0); /* ena */
 	_ppvar[1] = _nrn_mechanism_get_param_handle(prop_ion, 3); /* ina */
 	_ppvar[2] = _nrn_mechanism_get_param_handle(prop_ion, 4); /* _ion_dinadv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 {0, 0}
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
void _nrn_thread_table_reg(int, nrn_thread_table_check_t);
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 extern "C" void _na_rtm_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
 hoc_register_parm_default(_mechtype, &_parm_default);
         hoc_register_npy_direct(_mechtype, npy_direct_func_proc);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  register_nmodl_text_and_filename(_mechtype);
#endif
   _nrn_mechanism_register_data_fields(_mechtype,
                                       _nrn_mechanism_field<double>{"gbar"} /* 0 */,
                                       _nrn_mechanism_field<double>{"minfvhalf"} /* 1 */,
                                       _nrn_mechanism_field<double>{"minfk"} /* 2 */,
                                       _nrn_mechanism_field<double>{"hinfvhalf"} /* 3 */,
                                       _nrn_mechanism_field<double>{"hinfk"} /* 4 */,
                                       _nrn_mechanism_field<double>{"ina"} /* 5 */,
                                       _nrn_mechanism_field<double>{"minf"} /* 6 */,
                                       _nrn_mechanism_field<double>{"hinf"} /* 7 */,
                                       _nrn_mechanism_field<double>{"mtau"} /* 8 */,
                                       _nrn_mechanism_field<double>{"htau"} /* 9 */,
                                       _nrn_mechanism_field<double>{"g"} /* 10 */,
                                       _nrn_mechanism_field<double>{"m"} /* 11 */,
                                       _nrn_mechanism_field<double>{"h"} /* 12 */,
                                       _nrn_mechanism_field<double>{"ena"} /* 13 */,
                                       _nrn_mechanism_field<double>{"Dm"} /* 14 */,
                                       _nrn_mechanism_field<double>{"Dh"} /* 15 */,
                                       _nrn_mechanism_field<double>{"v"} /* 16 */,
                                       _nrn_mechanism_field<double>{"_g"} /* 17 */,
                                       _nrn_mechanism_field<double*>{"_ion_ena", "na_ion"} /* 0 */,
                                       _nrn_mechanism_field<double*>{"_ion_ina", "na_ion"} /* 1 */,
                                       _nrn_mechanism_field<double*>{"_ion_dinadv", "na_ion"} /* 2 */,
                                       _nrn_mechanism_field<int>{"_cvode_ieq", "cvodeieq"} /* 3 */);
  hoc_register_prop_size(_mechtype, 18, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 
    hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 na_rtm /Users/trophv/4001/mini-project-1/CI-BioEng-Class/fear_simulation/components/mechanisms/na_rtm.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static const char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rate(_internalthreadargsprotocomma_ double);
 
static int _ode_spec1(_internalthreadargsproto_);
/*static int _ode_matsol1(_internalthreadargsproto_);*/
 static neuron::container::field_index _slist1[1], _dlist1[1];
 static int states(_internalthreadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (_internalthreadargsproto_) {int _reset = 0; {
   rate ( _threadargscomma_ v ) ;
   Dh = ( hinf - h ) / htau ;
   }
 return _reset;
}
 static int _ode_matsol1 (_internalthreadargsproto_) {
 rate ( _threadargscomma_ v ) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / htau )) ;
  return 0;
}
 /*END CVODE*/
 static int states (_internalthreadargsproto_) { {
   rate ( _threadargscomma_ v ) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / htau)))*(- ( ( ( hinf ) ) / htau ) / ( ( ( ( - 1.0 ) ) ) / htau ) - h) ;
   }
  return 0;
}
 
static int  rate ( _internalthreadargsprotocomma_ double _lv ) {
    minf = 1.0 / ( 1.0 + ( exp ( ( _lv + minfvhalf ) / ( minfk ) ) ) ) ;
   mtau = ( 1.0 - exp ( - _lv / 4.0 - 27.0 / 2.0 ) ) * ( 1.0 - exp ( _lv / 5.0 + 27.0 / 5.0 ) ) / ( - ( 1.0 - exp ( - _lv / 4.0 - 27.0 / 2.0 ) ) * ( 0.28 * _lv + 7.56 ) + ( 1.0 - exp ( _lv / 5.0 + 27.0 / 5.0 ) ) * ( 0.32 * _lv + 17.28 ) ) ;
   hinf = 1.0 / ( 1.0 + ( exp ( ( _lv + hinfvhalf ) / ( hinfk ) ) ) ) ;
   htau = ( exp ( - _lv / 5.0 - 27.0 / 5.0 ) + 1.0 ) / ( 0.128 * ( exp ( - _lv / 5.0 - 27.0 / 5.0 ) + 1.0 ) * exp ( - _lv / 18.0 - 25.0 / 9.0 ) + 4.0 ) ;
     return 0; }
 
static void _hoc_rate(void) {
  double _r;
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 
  if(!_prop_id) {
    hoc_execerror("No data for rate_na_rtm. Requires prior call to setdata_na_rtm and that the specified mechanism instance still be in existence.", NULL);
  }
  Prop* _local_prop = _extcall_prop;
  _nrn_mechanism_cache_instance _ml_real{_local_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _local_prop ? _nrn_mechanism_access_dparam(_local_prop) : nullptr;
_thread = _extcall_thread.data();
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
_nt = nrn_threads;
 _r = 1.;
 rate ( _threadargscomma_ *getarg(1) );
 hoc_retpushx(_r);
}
 
static double _npy_rate(Prop* _prop) {
    double _r{0.0};
 Datum* _ppvar; Datum* _thread; NrnThread* _nt;
 _nrn_mechanism_cache_instance _ml_real{_prop};
auto* const _ml = &_ml_real;
size_t const _iml{};
_ppvar = _nrn_mechanism_access_dparam(_prop);
_thread = _extcall_thread.data();
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
_nt = nrn_threads;
 _r = 1.;
 rate ( _threadargscomma_ *getarg(1) );
 return(_r);
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
   Datum* _ppvar;
   size_t _iml;   _nrn_mechanism_cache_range* _ml;   Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
  ena = _ion_ena;
     _ode_spec1 (_threadargs_);
  }}
 
static void _ode_map(Prop* _prop, int _ieq, neuron::container::data_handle<double>* _pv, neuron::container::data_handle<double>* _pvdot, double* _atol, int _type) { 
  Datum* _ppvar;
  _ppvar = _nrn_mechanism_access_dparam(_prop);
  _cvode_ieq = _ieq;
  for (int _i=0; _i < 1; ++_i) {
    _pv[_i] = _nrn_mechanism_get_param_handle(_prop, _slist1[_i]);
    _pvdot[_i] = _nrn_mechanism_get_param_handle(_prop, _dlist1[_i]);
    _cvode_abstol(_atollist, _atol, _i);
  }
 }
 
static void _ode_matsol_instance1(_internalthreadargsproto_) {
 _ode_matsol1 (_threadargs_);
 }
 
static void _ode_matsol(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
   Datum* _ppvar;
   size_t _iml;   _nrn_mechanism_cache_range* _ml;   Node* _nd{};
  double _v{};
  int _cntml;
  _nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
  _ml = &_lmr;
  _cntml = _ml_arg->_nodecount;
  Datum *_thread{_ml_arg->_thread};
  double* _globals = nullptr;
  if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _ppvar = _ml_arg->_pdata[_iml];
    _nd = _ml_arg->_nodelist[_iml];
    v = NODEV(_nd);
  ena = _ion_ena;
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(_internalthreadargsproto_) {
  int _i; double _save;{
  h = h0;
  m = m0;
 {
   rate ( _threadargscomma_ v ) ;
   m = minf ;
   h = hinf ;
   }
 
}
}

static void nrn_init(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type){
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
 v = _v;
  ena = _ion_ena;
 initmodel(_threadargs_);
 }
}

static double _nrn_current(_internalthreadargsprotocomma_ double _v) {
double _current=0.; v=_v;
{ {
   m = minf ;
   g = gbar * m * m * m * h ;
   ina = g * ( v - ena ) ;
   }
 _current += ina;

} return _current;
}

static void nrn_cur(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_rhs = _nt->node_rhs_storage();
auto const _vec_sav_rhs = _nt->node_sav_rhs_storage();
auto const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (_iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
   _v = _vec_v[_ni[_iml]];
  ena = _ion_ena;
 auto const _g_local = _nrn_current(_threadargscomma_ _v + .001);
 	{ double _dina;
  _dina = ina;
 _rhs = _nrn_current(_threadargscomma_ _v);
  _ion_dinadv += (_dina - ina)/.001 ;
 	}
 _g = (_g_local - _rhs)/.001;
  _ion_ina += ina ;
	 _vec_rhs[_ni[_iml]] -= _rhs;
 
}
 
}

static void nrn_jacob(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto const _vec_d = _nt->node_d_storage();
auto const _vec_sav_d = _nt->node_sav_d_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
_ni = _ml_arg->_nodeindices;
_cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (_iml = 0; _iml < _cntml; ++_iml) {
  _vec_d[_ni[_iml]] += _g;
 
}
 
}

static void nrn_state(_nrn_model_sorted_token const& _sorted_token, NrnThread* _nt, Memb_list* _ml_arg, int _type) {
_nrn_mechanism_cache_range _lmr{_sorted_token, *_nt, *_ml_arg, _type};
auto* const _vec_v = _nt->node_voltage_storage();
auto* const _ml = &_lmr;
Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni;
_ni = _ml_arg->_nodeindices;
size_t _cntml = _ml_arg->_nodecount;
_thread = _ml_arg->_thread;
double* _globals = nullptr;
if (gind != 0 && _thread != nullptr) { _globals = _thread[_gth].get<double*>(); }
for (size_t _iml = 0; _iml < _cntml; ++_iml) {
 _ppvar = _ml_arg->_pdata[_iml];
 _nd = _ml_arg->_nodelist[_iml];
   _v = _vec_v[_ni[_iml]];
 v=_v;
{
  ena = _ion_ena;
 {   states(_threadargs_);
  } }}

}

static void terminal(){}

static void _initlists(){
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = {h_columnindex, 0};  _dlist1[0] = {Dh_columnindex, 0};
_first = 0;
}

#if NMODL_TEXT
static void register_nmodl_text_and_filename(int mech_type) {
    const char* nmodl_filename = "/Users/trophv/4001/mini-project-1/CI-BioEng-Class/fear_simulation/components/mechanisms/na_rtm.mod";
    const char* nmodl_file_text = 
  "COMMENT\n"
  "Reduced Traub-Miles Model of a Pyramidal\n"
  "Neuron in Rat Hippocampus\n"
  "\n"
  "Sodium Channel\n"
  "\n"
  "Reference: Borgers - An Introduction to Modeling Neuronal Dynamics Chapter 5\n"
  ".mod by Tyler Banks\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX na_rtm\n"
  "	USEION na READ ena WRITE ina\n"
  "	RANGE gbar, g\n"
  "	RANGE minf, hinf, mtau, htau\n"
  "	RANGE ina\n"
  "    RANGE minfvhalf,minfk,hinfvhalf,hinfk\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mA) = (milliamp)\n"
  "	(mV) = (millivolt)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	gbar (siemens/cm2)\n"
  "    minfvhalf = 41.03\n"
  "    minfk = -7.1 :-7.41\n"
  "    hinfvhalf = 45.32\n"
  "    hinfk = 4.04\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v (mV)\n"
  "	ena (mV)\n"
  "	ina (mA/cm2)\n"
  "	minf\n"
  "	hinf\n"
  "	mtau (ms)\n"
  "	htau (ms)\n"
  "	g (siemens/cm2)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	m h\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE states METHOD cnexp\n"
  "	m = minf : See Borgers Page 32 Figure 5.1 for explaination\n"
  "	g = gbar*m*m*m*h\n"
  "	ina = g*(v-ena)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	rate(v)\n"
  "	m = minf\n"
  "	h = hinf\n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "	rate(v)\n"
  "	:m' = (minf-m)/mtau\n"
  "	h' = (hinf-h)/htau\n"
  "}\n"
  "\n"
  "COMMENT\n"
  "\n"
  "function m_e_inf=m_e_inf(v)\n"
  "alpha_m=0.32*(v+54)./(1-exp(-(v+54)/4));\n"
  "beta_m=0.28*(v+27)./(exp((v+27)/5)-1);\n"
  "m_e_inf=alpha_m./(alpha_m+beta_m);\n"
  ":minf = 0.32*(v+54)/(1-exp(-(v+54)/4))/(0.32*(v+54)/(1-exp(-(v+54)/4))+0.28*(v+27)/(exp((v+27)/5)-1))\n"
  ":mtau = 1/(0.32*(v+54)/(1-exp(-(v+54)/4))+0.28*(v+27)/(exp((v+27)/5)-1))\n"
  "\n"
  "function h_e_inf=h_e_inf(v)\n"
  "alpha_h=0.128*exp(-(v+50)/18);\n"
  "beta_h=4./(1+exp(-(v+27)/5));\n"
  "h_e_inf=alpha_h./(alpha_h+beta_h);\n"
  ":hinf = 0.128*exp(-(v+50)/18)/(0.128*exp(-(v+50)/18)+4/(1+exp(-(v+27)/5)))\n"
  ":htau = 1/(0.128*exp(-(v+50)/18)+4/(1+exp(-(v+27)/5)))\n"
  "\n"
  "Regression fit INF\n"
  "minf = 1.0/(1.0+(exp((v+41.03)/(-7.41))))\n"
  "hinf = 1.0/(1.0+(exp((v+45.32)/(4.04))))\n"
  "\n"
  "Calculated TAU\n"
  "mtau = (1 - exp(-v/4 - 27/2))*(1 - exp(v/5 + 27/5))/(-(1 - exp(-v/4 - 27/2))*(0.28*v + 7.56) + (1 - exp(v/5 + 27/5))*(0.32*v + 17.28))\n"
  "htau = (exp(-v/5 - 27/5) + 1)/(0.128*(exp(-v/5 - 27/5) + 1)*exp(-v/18 - 25/9) + 4)\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "PROCEDURE rate(v (mV)) {\n"
  "	UNITSOFF\n"
  "	minf = 1.0/(1.0+(exp((v+minfvhalf)/(minfk))))\n"
  "	:minf = 0.32*(v+54)/(1-exp(-(v+54)/4))/(0.32*(v+54)/(1-exp(-(v+54)/4))+0.28*(v+27)/(exp((v+27)/5)-1))\n"
  "	mtau = (1 - exp(-v/4 - 27/2))*(1 - exp(v/5 + 27/5))/(-(1 - exp(-v/4 - 27/2))*(0.28*v + 7.56) + (1 - exp(v/5 + 27/5))*(0.32*v + 17.28))   \n"
  "	:mtau = 1/(0.32*(v+54)/(1-exp(-(v+54)/4))+0.28*(v+27)/(exp((v+27)/5)-1))\n"
  "\n"
  "	hinf = 1.0/(1.0+(exp((v+hinfvhalf)/(hinfk))))  \n"
  "	:hinf = 0.128*exp(-(v+50)/18)/(0.128*exp(-(v+50)/18)+4/(1+exp(-(v+27)/5)))\n"
  "	htau = (exp(-v/5 - 27/5) + 1)/(0.128*(exp(-v/5 - 27/5) + 1)*exp(-v/18 - 25/9) + 4)\n"
  "	:htau = 1/(0.128*exp(-(v+50)/18)+4/(1+exp(-(v+27)/5)))\n"
  "	UNITSON\n"
  "}\n"
  ;
    hoc_reg_nmodl_filename(mech_type, nmodl_filename);
    hoc_reg_nmodl_text(mech_type, nmodl_file_text);
}
#endif
