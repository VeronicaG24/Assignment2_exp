/* BISON HDDL grammar file */

/* use modern C++ (14) */
%skeleton "lalr1.cc"

/* use classes */
%define api.token.constructor
%define api.value.type variant

/* needed by modern C++ (14) */
%define api.token.prefix {T_}

%code requires {
    #include <string>

    class HDDLParser;
}

// The parsing context
%param { HDDLParser& hddl_parser }

// enable yy::location location_; used by parser to communicate the file path
%locations

// enable verbose parser output
%define parse.trace
%define parse.error verbose

%code {
    #include <fstream>
    #include <sstream>
    #include "hddl_parser.h"

    Predicate temp_predicate;
    std::vector<Predicate> temp_pred_vector;
    Params temp_params;
    Method temp_meth;
    Action temp_action;
    std::vector<std::string> temp_args;
    std::vector<std::string> temp_instances;
    int count = 0;
}

%token <int>
    NUMBER                  " number "

%token <std::string>
    STRING                  " generic string "
    DOMAIN                  " domain "
    DEFINE                  " define "
    PROBLEM                 " problem "

    HDDL_REQ_KEYWORD        " requirements "
    HDDL_REQ_PARAM          " typing, htn, htn-method-prec "

    HDDL_TYPES_KEYWORD      " types "
    HDDL_PRED_KEYWORD       " predicates "
    HDDL_PARAMS_KEYWORD     " parameters "
    HDDL_OP_PREC_KEYWORD    " precondition "
    HDDL_OP_DUR_KEYWORD     " duration "
    HDDL_OP_EFF_KEYWORD     " effect "
    HDDL_OP_DEC_KEYWORD     " decrease "
    HDDL_TASK_KEYWORD       " task "
    HDDL_ORD_SUBT_KEYWORD   " ordered-subtasks "
    HDDL_SUBT_KEYWORD       " subtasks "
    HDDL_METHOD_KEYWORD     " method "
    HDDL_ACTION_KEYWORD     " action "

    LPAREN                  " ( "
    COLON                   " : "
    RPAREN                  " ) "
    QM                      " ? "
    HYPHEN                  " - "
    EQUAL                   " = "
    AND                     " and "
    NOT                     " not "

%token
    END  0                  " end_of_file "
    OTHER                   "invalid character"

/* =============================

   Grammar definition starts here

================================*/

%%

%start hddl_main_structure;

hddl_main_structure:
    /* (define (domain foo_domain_name) */
    LPAREN DEFINE LPAREN DOMAIN STRING RPAREN { hddl_parser.domain_.domain_name_ = $5; }

    /* (:requirements :typing :durative-actions) */
    requirements

    /* (:types robot location) */
    types

    /* (:predicates pred1 pred2) ; pred1 e.g. (pred_name ?r - robot ?l - location) */
    predicates

    /* (:task deliver :parameters (?p - package ?l - location)) */
    tasks

    /* (:method m-deliver ... */
    methods

    /* (:action drive */
    actions

    /* final closing parenthesis */
    RPAREN

/* (:requirements req1 req2) */
requirements:
    LPAREN COLON HDDL_REQ_KEYWORD reqs RPAREN

/* allow multiple requirements of the form -> :req1 :req2 */
reqs:
    | reqs req

/* e.g :typing */
req:
    COLON HDDL_REQ_PARAM { hddl_parser.domain_.domain_requirements_.push_back($2); }

/* e.g. (:types kitchen bedroom - location) */
types:
    LPAREN COLON HDDL_TYPES_KEYWORD types_content RPAREN

/* allow multiple instances - types, e.g. kitchen bedroom - location /n kenny youbot - robot */
types_content:
    | types_content instances HYPHEN STRING
    {
        while (temp_instances.size() > 0) {
            hddl_parser.domain_.domain_types_[temp_instances.back()] = $4;
            temp_instances.pop_back();
        }
    }

/* allow multiple strings (instances) e.g. kitchen bedroom */
instances:
    | instances STRING { temp_instances.push_back($2);}

/* (:predicates pred1 pred2) */
predicates:
    LPAREN COLON HDDL_PRED_KEYWORD hpreds RPAREN

/* allow multiple predicates, e.g. (robot_at ?r - robot ?l - location) */
hpreds:
    | hpreds LPAREN STRING only_params RPAREN {
        temp_predicate.name = $3;
        temp_predicate.pred_params = temp_params;
        hddl_parser.domain_.domain_predicates_.push_back(temp_predicate);

        /* reset */
        temp_params.params.clear();
        temp_params.params_map.clear();
        count = 0;
    }

/* :parameters (?p - package ?l1 ?l2 - location ?v - vehicle) */
params:
    COLON HDDL_PARAMS_KEYWORD LPAREN only_params RPAREN

/* ?p - package ?l1 ?l2 - location ?v - vehicle */
only_params:
    | only_params keys HYPHEN STRING {
        for(int i=0; i < count; i++) {
            temp_params.params_map[temp_args.at(i)] = $4;
        }

        temp_args.clear();
        count = 0;
    }

/* allow multiple keys, e.g. ?foo ?bar */
keys:
    | keys QM STRING {
        count++;
        temp_args.push_back($3);
        temp_params.params.push_back($3);
    }

/* allow multiple tasks e.g. task1 /n task2 /n task3 */
tasks:
    task | tasks task

/* e.g. (:task deliver :parameters (?p - package ?l - location)) */
task:
    LPAREN COLON HDDL_TASK_KEYWORD STRING params RPAREN {
        Task temp_task;
        temp_task.name = $4;
        temp_task.task_params = temp_params;
        hddl_parser.domain_.domain_tasks_.push_back(temp_task);

        /* reset params */
        temp_params.params.clear();
        temp_params.params_map.clear();
        count = 0;
    }

methods:
    method | methods method

method:
    /*
     (:method
      :parameters (?p - package ?l1 ?l2 - location ?v - vehicle)
      :task (deliver ?p ?l2)
      ...
     )
     */
    LPAREN COLON HDDL_METHOD_KEYWORD STRING params meth_task subtasks RPAREN {
        temp_meth.name = $4;
        temp_meth.meth_params = temp_params;
        hddl_parser.domain_.domain_meths_.push_back(temp_meth);

        temp_meth.subtasks.clear();

        /* reset */
        temp_params.params.clear();
        temp_params.params_map.clear();
        count = 0;
    }

meth_task:
    /* :task (deliver ?p ?l2) */
    COLON HDDL_TASK_KEYWORD LPAREN STRING simple_params RPAREN {
        /* parameters */
        temp_meth.task.name = $4;
        temp_meth.task.task_params.params = temp_args;
        temp_args.clear();
    }

simple_params:
    | simple_params QM STRING {
        temp_args.push_back($3);
    }

subtasks:
    /*   :ordered-subtasks (and ... )
         or
         :subtasks (and ... ) */
    COLON ordered meth_sub

meth_sub:
    /* :subtasks (drop ?v ?l ?p ?s1 ?s2) */
    LPAREN AND subts RPAREN | subt

ordered:
    HDDL_ORD_SUBT_KEYWORD { temp_meth.ordered_subtasks = true; } | HDDL_SUBT_KEYWORD { temp_meth.ordered_subtasks = false; }

subts:
    | subts subt

subt:
    LPAREN STRING simple_params RPAREN {
        Task t;
        t.name = $2;
        t.task_params.params = temp_args;
        temp_meth.subtasks.push_back(t);

        temp_args.clear();
    }

actions:
    action | actions action

action:
/* (:action drive
    :parameters (?v - vehicle ?l1 ?l2 - location) */
    LPAREN COLON HDDL_ACTION_KEYWORD STRING params {temp_action.action_params = temp_params;} preconditions effects RPAREN {
        temp_action.name = $4;

        hddl_parser.domain_.domain_actions_.push_back(temp_action);

        temp_action.effects.clear();
        temp_action.preconditions.clear();

        /* reset params */
        temp_params.params.clear();
        temp_params.params_map.clear();
        count = 0;
    }

preconditions:
/*
 :precondition (and
        (at ?v ?l1)
        (road ?l1 ?l2)) */
    COLON HDDL_OP_PREC_KEYWORD LPAREN RPAREN | COLON HDDL_OP_PREC_KEYWORD pred_body {
        temp_action.preconditions = temp_pred_vector;
        temp_pred_vector.clear();
    }

effects:
/*
 :effect (and
        (not (at ?v ?l1))
        (at ?v ?l2)) */
    COLON HDDL_OP_EFF_KEYWORD LPAREN RPAREN | COLON HDDL_OP_EFF_KEYWORD pred_body {
        temp_action.effects = temp_pred_vector;
        temp_pred_vector.clear();
    }

pred_body:
    LPAREN AND preds RPAREN | pred

preds:
    pred | preds pred

pred:
    positive_pred | negative_pred

positive_pred:
    LPAREN STRING keys RPAREN {
        temp_predicate.negated = false;

        temp_predicate.name = $2;
        temp_predicate.pred_params.params = temp_args;

        temp_pred_vector.push_back(temp_predicate);

        temp_args.clear();
    }

negative_pred:
    LPAREN NOT LPAREN STRING keys RPAREN RPAREN {
        temp_predicate.negated = true;

        temp_predicate.name = $4;
        temp_predicate.pred_params.params = temp_args;

        temp_pred_vector.push_back(temp_predicate);

        temp_args.clear();
    }

%%

/* =============================

   Grammar definition ends here

================================*/

void yy::parser::error(const location_type& error_location, const std::string& m)
{
    // show error to the user: 1. type of error , 2. filename , 3. error location (line and column)
    std::cerr << "\n\033[1;31merror: \033[0m" << m << "\nwhile parsing file: " << *error_location.begin.filename << \
    " line : " << error_location.begin.line << ", column : " << error_location.begin.column \
    << ":" << error_location.end.column << "\n" << std::endl;

    // print the file to console and mark where the error is with a "^"
    std::ifstream file(error_location.begin.filename->c_str());
    if (file.is_open())
    {
        std::string line;
        int line_number = 0;
        std::cout << "========  " << *error_location.begin.filename << "  ========" << std::endl;
        // print all lines one by one
        while(std::getline(file, line))
        {
            // print "^" chars to mark where the error is located
            if(error_location.begin.line == line_number++)
            {
                std::stringstream ss_marker;
                // add spaces to string stream marker until error column starts
                for(size_t i = 1; i < error_location.begin.column; i++)
                    ss_marker << " ";

                // add ^ chars from the beginning of the column till end of error
                for(size_t j = error_location.begin.column; j < (error_location.end.column); j++)
                    ss_marker << "^";

                // print line that shows where the error is found
                // std::cout << ss_marker.str() << std::endl;
                std::cout << "\033[1;32m" << ss_marker.str() << "\033[0m" << std::endl;
            }

            // print next line
            std::istringstream ss(line);
            std::cout << ss.str() << std::endl;
        }
        std::cout << "=======================================" << std::endl;
    }
}
