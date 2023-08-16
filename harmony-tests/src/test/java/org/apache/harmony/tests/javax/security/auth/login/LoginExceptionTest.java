/*
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.
 *  The ASF licenses this file to You under the Apache License, Version 2.0
 *  (the "License"); you may not use this file except in compliance with
 *  the License.  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

package org.apache.harmony.tests.javax.security.auth.login;

import junit.framework.TestCase;

import javax.security.auth.callback.PasswordCallback;
import javax.security.auth.login.LoginException;
import org.apache.harmony.testframework.serialization.SerializationTest;

/**
 * Tests for <code>LoginException</code> class constructors and methods.
 *
 */
public class LoginExceptionTest extends TestCase {

    private static String[] msgs = {
            "",
            "Check new message",
            "Check new message Check new message Check new message Check new message Check new message" };


    /**
     * javax.security.auth.login.LoginException#LoginException()
     * Assertion: constructs LoginException with no detail message
     */
    public void testLoginException01() {
        LoginException lE = new LoginException();
        assertNull("getMessage() must return null.", lE.getMessage());
        assertNull("getCause() must return null", lE.getCause());
    }

    /**
     * javax.security.auth.login.LoginException#LoginException(String msg)
     * Assertion: constructs with not null parameter.
     */
    public void testLoginException02() {
        LoginException lE;
        for (int i = 0; i < msgs.length; i++) {
            lE = new LoginException(msgs[i]);
            assertEquals("getMessage() must return: ".concat(msgs[i]), lE.getMessage(), msgs[i]);
            assertNull("getCause() must return null", lE.getCause());
        }
    }

    /**
     * javax.security.auth.login.LoginException#LoginException(String msg)
     * Assertion: constructs with null parameter.
     */
    public void testLoginException03() {
        String msg = null;
        LoginException lE = new LoginException(msg);
        assertNull("getMessage() must return null.", lE.getMessage());
        assertNull("getCause() must return null", lE.getCause());
    }

    public void testSerializationSelf() throws Exception {
        SerializationTest.verifySelf(getSerializationData());
    }

    public void testSerializationGolden() throws Exception {
        SerializationTest.verifyGolden(this, getSerializationData());
    }

    private Object[] getSerializationData() {
        return new Object[] { new LoginException("message") };
    }
}
