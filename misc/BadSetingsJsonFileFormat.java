package org.xero1425.misc;

public class BadSetingsJsonFileFormat extends Exception {
    private String param_ ;

    static final long serialVersionUID = 42 ;
    
    /// \brief create the exception object
    /// \param param the name of the missing parmater
    public BadSetingsJsonFileFormat(String param) {
        super("JSON for parameter '" + param + "' was malformed, expected value, got object or array") ;
        param_ = param ;
    }

    /// \brief returns the name of the missing parameter
    /// \returns the name of the missing parameter
    public String getParameter() {
        return param_ ;
    }
}

