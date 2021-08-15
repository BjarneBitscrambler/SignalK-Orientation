BEGIN {
	print "Syntax:  gawk -f curl_export.awk file_of_value_pairs.csv";
	print "Reads a CSV file, consisting of Number,Number pairs, and prints the curl command";
	print "that will perform an HTTP PUT to the eCompass to store them as a deviation table.";
	print "Modify the IP address in this script to match your sensor's. Check that the config path matches your sensor's web interface.";
	print "Run the command by pasting the result to the linux command line of a computer connected to the same network as your sensor.  The success of the operation can be confirmed by examining the result_file; it should state 'Configuration successful.'";
	FS = ",";
	preamble = "curl -X PUT -H \"Content-Type: application/json\" -d '{\"samples\":[";
	postamble = "]}' http://192.168.1.10/config/sensors/heading/deviation > result_file";
	values = "";
	firstRecord = 1;
}
{
    if( NF == 2 ) {
    	if( 1 == firstRecord ) {
    		values = values "{\"input\":" $1 ",\"output\":" $2 "}";
    		firstRecord = 0;
    	}else{
    		values = values ",{\"input\":" $1 ",\"output\":" $2 "}";
    	}
    }else{
    	print "Found " NF " fields in record " NR " but expected 2.";
    }
    
}
END {
	print "Done. Command follows:";
	print preamble values postamble;
}
