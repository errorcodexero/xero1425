package org.xero1425.misc;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

public class JsonSettingsParser implements ISettingsSupplier {
    private MessageLogger logger_ ;
    private List<String> defines_ ;
    private JSONObject contents_ ;

    public JsonSettingsParser(MessageLogger logger) {
        logger_ = logger ;
        defines_ = new ArrayList<String>() ;
    }

    public void addDefine(String name) {
        if (!defines_.contains(name))
            defines_.add(name) ;
    }

    public boolean readFile(String filename) {

        logger_.startMessage(MessageType.Info);
        logger_.add("reading JSON robots setting file ").addQuoted(filename) ;
        logger_.endMessage();        

        byte[] encoded;
        try {
            encoded = Files.readAllBytes(Paths.get(filename));
        } catch (IOException e) {
            logger_.startMessage(MessageType.Error);
            logger_.add("cannot read settings file ").addQuoted(filename).add(" - ");
            logger_.add(e.getMessage()).endMessage();
            return false;
        }

        String fulltext = new String(encoded);

        Object obj = JSONValue.parse(fulltext);
        if (!(obj instanceof JSONObject)) {
            logger_.startMessage(MessageType.Error);
            logger_.add("cannot read settings file ").addQuoted(filename).add(" - ");
            logger_.add("file does not contain a JSON object").endMessage();
            return false;
        }

        contents_ = (JSONObject) obj;
        return true;        
    }

    @Override
    public SettingsValue get(String name) throws MissingParameterException {
        SettingsValue v = getOrNull(name) ;

        if (v == null)
            throw new MissingParameterException(name) ;

        return v ;
    }

    @Override
    public boolean isDefined(String name) {
        SettingsValue v = getOrNull(name) ;
        return v != null ;
    }

    @Override
    public SettingsValue getOrNull(String name) {
        String [] parts = name.split(":") ;
        SettingsValue v = null ;

        JSONObject current = findParent(parts) ;
        if (current == null)
            return null ;

        Object value = current.get(parts[parts.length-1]) ;
        if (value == null)
            return null ;

        if ((value instanceof JSONObject)) {
            //
            // This might be a conditional definiton based on a define
            //
            JSONObject condobj = (JSONObject)value ;
            for(String define : defines_)
            {
                if (condobj.containsKey(define))
                {
                    value = condobj.get(define) ;
                    break ;
                }
            }                
        }

        if ((value instanceof Double) == true) {
            v = new SettingsValue((Double)value) ;
        }
        else if ((value instanceof Integer) == true) {
            v = new SettingsValue((Integer)value) ;
        }
        else if ((value instanceof Long) == true) {
            v = new SettingsValue((Long)value) ;
        }
        else if ((value instanceof Boolean) == true) {
            v = new SettingsValue((Boolean)value) ;
        }
        else if ((value instanceof String) == true) {
            v = new SettingsValue((String)value) ;
        }

        return v;
    }

    @Override
    public List<String> getAllKeys(String path) {
        String [] parts = path.split(":") ;
        JSONObject parent = findParent(parts) ;

        List<String> ret = new ArrayList<String>() ;

        JSONObject obj = (JSONObject)parent.get(path) ;
        if (obj != null) {
            @SuppressWarnings("unchecked")
            Set<String> keys = (Set<String>)obj.keySet() ;
            if (keys != null) {
                for(String key : keys)
                    ret.add(key) ;
            }
        }

        return ret ;
    }

    private JSONObject findParent(String [] parts) {
        JSONObject current = contents_ ;
        int index = 0 ;

        while (index < parts.length - 1)
        {
            Object obj = current.get(parts[index]) ;
            if (obj == null)
                return null ;

            if (!(obj instanceof JSONObject))
                return null ;

            current = (JSONObject)obj ;
            index++ ;
        }

        return current ;
    }
}
