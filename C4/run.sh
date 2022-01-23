#!/bin/bash

challenge="4"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:p:r:h:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    4)
        # how to call agent for challenge 4
        python3 mainC4.py -h "$host" -p "$pos" -r "$robname" -f "$outfile"
        ;;
        # how to call agent for challenge 4
        #java mainC4 -h "$host" -p "$pos" -r "$robname" 
        #mv your_mapfile $outfile.map
        #mv your_pathfile $outfile.path
        #;;

        # how to call agent for challenge 4
        #./mainC4 -h "$host" -p "$pos" -r "$robname"
        #mv your_mapfile $outfile.map
        #mv your_pathfile $outfile.path
        #;;
esac

