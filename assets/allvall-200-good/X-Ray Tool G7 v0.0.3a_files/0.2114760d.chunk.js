(this["webpackJsonpstreamlit-browser"]=this["webpackJsonpstreamlit-browser"]||[]).push([[0],{2727:function(e,t,r){"use strict";r.d(t,"g",(function(){return c})),r.d(t,"f",(function(){return u})),r.d(t,"e",(function(){return p})),r.d(t,"j",(function(){return b})),r.d(t,"d",(function(){return f})),r.d(t,"c",(function(){return h})),r.d(t,"h",(function(){return g})),r.d(t,"b",(function(){return y})),r.d(t,"i",(function(){return v})),r.d(t,"a",(function(){return m}));var o=r(28),n=r(89),i=r(419);function a(e,t){var r=Object.keys(e);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);t&&(o=o.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),r.push.apply(r,o)}return r}function l(e){for(var t=1;t<arguments.length;t++){var r=null!=arguments[t]?arguments[t]:{};t%2?a(Object(r),!0).forEach((function(t){s(e,t,r[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(r)):a(Object(r)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(r,t))}))}return e}function s(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}var c=Object(o.a)("button",(function(e){var t,r=e.$theme,o=e.$size,i=e.$isFocusVisible,a=(t={},s(t,n.d.mini,r.sizing.scale400),s(t,n.d.compact,r.sizing.scale400),s(t,n.d.default,r.sizing.scale300),s(t,n.d.large,r.sizing.scale200),t)[o];return{display:"flex",alignItems:"center",borderTopStyle:"none",borderBottomStyle:"none",borderLeftStyle:"none",borderRightStyle:"none",background:"none",paddingLeft:a,paddingRight:a,outline:i?"solid 3px ".concat(r.colors.accent):"none",color:r.colors.contentPrimary}}));c.displayName="StyledMaskToggleButton";var u=Object(o.a)("div",(function(e){var t,r=e.$alignTop,o=void 0!==r&&r,i=e.$size,a=e.$theme,l=(t={},s(t,n.d.mini,a.sizing.scale200),s(t,n.d.compact,a.sizing.scale200),s(t,n.d.default,a.sizing.scale100),s(t,n.d.large,a.sizing.scale0),t)[i];return{display:"flex",alignItems:o?"flex-start":"center",paddingLeft:l,paddingRight:l,paddingTop:o?a.sizing.scale500:"0px",color:a.colors.contentPrimary}}));u.displayName="StyledClearIconContainer";var p=Object(o.a)(i.a,(function(e){var t=e.$theme;return{cursor:"pointer",outline:e.$isFocusVisible?"solid 3px ".concat(t.colors.accent):"none"}}));function d(e,t){var r;return(r={},s(r,n.d.mini,t.font100),s(r,n.d.compact,t.font200),s(r,n.d.default,t.font300),s(r,n.d.large,t.font400),r)[e]}p.displayName="StyledClearIcon";var b=function(e){var t,r=e.$isFocused,o=e.$adjoined,i=e.$error,a=e.$disabled,s=e.$positive,c=e.$size,u=e.$theme,p=e.$theme,b=p.borders,f=p.colors,h=p.sizing,g=p.typography,y=p.animation,v=e.$hasIconTrailing;return l({boxSizing:"border-box",display:"flex",overflow:"hidden",width:"100%",borderLeftWidth:"2px",borderRightWidth:"2px",borderTopWidth:"2px",borderBottomWidth:"2px",borderLeftStyle:"solid",borderRightStyle:"solid",borderTopStyle:"solid",borderBottomStyle:"solid",transitionProperty:"border",transitionDuration:y.timing200,transitionTimingFunction:y.easeOutCurve},{borderTopLeftRadius:t=b.inputBorderRadius,borderBottomLeftRadius:t,borderTopRightRadius:t,borderBottomRightRadius:t},{},d(c,g),{},function(e,t,r,o,n){return e?{borderLeftColor:n.inputFillDisabled,borderRightColor:n.inputFillDisabled,borderTopColor:n.inputFillDisabled,borderBottomColor:n.inputFillDisabled,backgroundColor:n.inputFillDisabled}:t?{borderLeftColor:n.borderFocus,borderRightColor:n.borderFocus,borderTopColor:n.borderFocus,borderBottomColor:n.borderFocus,backgroundColor:n.inputFillActive}:r?{borderLeftColor:n.inputBorderError,borderRightColor:n.inputBorderError,borderTopColor:n.inputBorderError,borderBottomColor:n.inputBorderError,backgroundColor:n.inputFillError}:o?{borderLeftColor:n.inputBorderPositive,borderRightColor:n.inputBorderPositive,borderTopColor:n.inputBorderPositive,borderBottomColor:n.inputBorderPositive,backgroundColor:n.inputFillPositive}:{borderLeftColor:n.inputBorder,borderRightColor:n.inputBorder,borderTopColor:n.inputBorder,borderBottomColor:n.inputBorder,backgroundColor:n.inputFill}}(a,r,i,s,f),{},function(e,t,r,o,i){var a=e===n.a.both||e===n.a.left&&"rtl"!==o||e===n.a.right&&"rtl"===o||i&&"rtl"===o,l=e===n.a.both||e===n.a.right&&"rtl"!==o||e===n.a.left&&"rtl"===o||i&&"rtl"!==o;return{paddingLeft:a?r.scale550:"0px",paddingRight:l?r.scale550:"0px"}}(o,0,h,u.direction,v))},f=Object(o.a)("div",b);f.displayName="Root";var h=Object(o.a)("div",(function(e){var t=e.$size,r=e.$disabled,o=e.$isFocused,i=e.$error,a=e.$positive,c=e.$theme,u=c.colors,p=c.sizing,b=c.typography,f=c.animation;return l({display:"flex",alignItems:"center",justifyContent:"center",transitionProperty:"color, background-color",transitionDuration:f.timing200,transitionTimingFunction:f.easeOutCurve},d(t,b),{},function(e,t){var r;return(r={},s(r,n.d.mini,{paddingRight:t.scale400,paddingLeft:t.scale400}),s(r,n.d.compact,{paddingRight:t.scale400,paddingLeft:t.scale400}),s(r,n.d.default,{paddingRight:t.scale300,paddingLeft:t.scale300}),s(r,n.d.large,{paddingRight:t.scale200,paddingLeft:t.scale200}),r)[e]}(t,p),{},function(e,t,r,o,n){return e?{color:n.inputEnhancerTextDisabled,backgroundColor:n.inputFillDisabled}:t?{color:n.contentPrimary,backgroundColor:n.inputFillActive}:r?{color:n.contentPrimary,backgroundColor:n.inputFillError}:o?{color:n.contentPrimary,backgroundColor:n.inputFillPositive}:{color:n.contentPrimary,backgroundColor:n.inputFill}}(r,o,i,a,u))}));h.displayName="InputEnhancer";var g=function(e){var t=e.$isFocused,r=e.$error,o=e.$disabled,n=e.$positive,i=e.$size,a=e.$theme,s=a.colors,c=a.typography,u=a.animation;return l({display:"flex",width:"100%",transitionProperty:"background-color",transitionDuration:u.timing200,transitionTimingFunction:u.easeOutCurve},d(i,c),{},function(e,t,r,o,n){return e?{color:n.inputTextDisabled,backgroundColor:n.inputFillDisabled}:t?{color:n.contentPrimary,backgroundColor:n.inputFillActive}:r?{color:n.contentPrimary,backgroundColor:n.inputFillError}:o?{color:n.contentPrimary,backgroundColor:n.inputFillPositive}:{color:n.contentPrimary,backgroundColor:n.inputFill}}(o,t,r,n,s))},y=Object(o.a)("div",g);y.displayName="InputContainer";var v=function(e){var t=e.$disabled,r=(e.$isFocused,e.$error,e.$size),o=e.$theme,i=o.colors,a=o.sizing;return l({boxSizing:"border-box",backgroundColor:"transparent",borderLeftWidth:0,borderRightWidth:0,borderTopWidth:0,borderBottomWidth:0,borderLeftStyle:"none",borderRightStyle:"none",borderTopStyle:"none",borderBottomStyle:"none",outline:"none",width:"100%",maxWidth:"100%",cursor:t?"not-allowed":"text",margin:"0",paddingTop:"0",paddingBottom:"0",paddingLeft:"0",paddingRight:"0"},d(r,o.typography),{},function(e,t){var r;return(r={},s(r,n.d.mini,{paddingTop:t.scale100,paddingBottom:t.scale100,paddingLeft:t.scale550,paddingRight:t.scale550}),s(r,n.d.compact,{paddingTop:t.scale200,paddingBottom:t.scale200,paddingLeft:t.scale550,paddingRight:t.scale550}),s(r,n.d.default,{paddingTop:t.scale400,paddingBottom:t.scale400,paddingLeft:t.scale550,paddingRight:t.scale550}),s(r,n.d.large,{paddingTop:t.scale550,paddingBottom:t.scale550,paddingLeft:t.scale550,paddingRight:t.scale550}),r)[e]}(r,a),{},function(e,t,r,o){return e?{color:o.inputTextDisabled,caretColor:o.contentPrimary,"::placeholder":{color:o.inputPlaceholderDisabled}}:{color:o.contentPrimary,caretColor:o.contentPrimary,"::placeholder":{color:o.inputPlaceholder}}}(t,0,0,i))},m=Object(o.a)("input",v);m.displayName="Input"},2814:function(e,t,r){"use strict";function o(e,t){var r=e.disabled,o=e.error,n=e.positive,i=e.adjoined,a=e.size,l=e.required;return{$isFocused:t.isFocused,$disabled:r,$error:o,$positive:n,$adjoined:i,$size:a,$required:l}}r.d(t,"a",(function(){return o}))},2906:function(e,t,r){"use strict";var o=r(0),n=r(18),i=r(89),a=r(2727),l=r(2814),s=r(28),c=r(72);function u(){return(u=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var o in r)Object.prototype.hasOwnProperty.call(r,o)&&(e[o]=r[o])}return e}).apply(this,arguments)}function p(e,t){if(null==e)return{};var r,o,n=function(e,t){if(null==e)return{};var r,o,n={},i=Object.keys(e);for(o=0;o<i.length;o++)r=i[o],t.indexOf(r)>=0||(n[r]=e[r]);return n}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(o=0;o<i.length;o++)r=i[o],t.indexOf(r)>=0||Object.prototype.propertyIsEnumerable.call(e,r)&&(n[r]=e[r])}return n}function d(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){if(!(Symbol.iterator in Object(e))&&"[object Arguments]"!==Object.prototype.toString.call(e))return;var r=[],o=!0,n=!1,i=void 0;try{for(var a,l=e[Symbol.iterator]();!(o=(a=l.next()).done)&&(r.push(a.value),!t||r.length!==t);o=!0);}catch(s){n=!0,i=s}finally{try{o||null==l.return||l.return()}finally{if(n)throw i}}return r}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance")}()}var b=o.forwardRef((function(e,t){var r=d(Object(s.b)(),2)[1],i=e.title,a=void 0===i?"Hide":i,l=e.size,b=e.color,f=e.overrides,h=void 0===f?{}:f,g=p(e,["title","size","color","overrides"]),y=Object(n.d)({component:r.icons&&r.icons.Hide?r.icons.Hide:null},h&&h.Svg?Object(n.f)(h.Svg):{});return o.createElement(c.a,u({viewBox:"0 0 20 20",ref:t,title:a,size:l,color:b,overrides:{Svg:y}},g),o.createElement("path",{d:"M12.81 4.36l-1.77 1.78a4 4 0 00-4.9 4.9l-2.76 2.75C2.06 12.79.96 11.49.2 10a11 11 0 0112.6-5.64zm3.8 1.85c1.33 1 2.43 2.3 3.2 3.79a11 11 0 01-12.62 5.64l1.77-1.78a4 4 0 004.9-4.9l2.76-2.75zm-.25-3.99l1.42 1.42L3.64 17.78l-1.42-1.42L16.36 2.22z"}))}));function f(){return(f=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var o in r)Object.prototype.hasOwnProperty.call(r,o)&&(e[o]=r[o])}return e}).apply(this,arguments)}function h(e,t){if(null==e)return{};var r,o,n=function(e,t){if(null==e)return{};var r,o,n={},i=Object.keys(e);for(o=0;o<i.length;o++)r=i[o],t.indexOf(r)>=0||(n[r]=e[r]);return n}(e,t);if(Object.getOwnPropertySymbols){var i=Object.getOwnPropertySymbols(e);for(o=0;o<i.length;o++)r=i[o],t.indexOf(r)>=0||Object.prototype.propertyIsEnumerable.call(e,r)&&(n[r]=e[r])}return n}function g(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){if(!(Symbol.iterator in Object(e))&&"[object Arguments]"!==Object.prototype.toString.call(e))return;var r=[],o=!0,n=!1,i=void 0;try{for(var a,l=e[Symbol.iterator]();!(o=(a=l.next()).done)&&(r.push(a.value),!t||r.length!==t);o=!0);}catch(s){n=!0,i=s}finally{try{o||null==l.return||l.return()}finally{if(n)throw i}}return r}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance")}()}var y=o.forwardRef((function(e,t){var r=g(Object(s.b)(),2)[1],i=e.title,a=void 0===i?"Show":i,l=e.size,u=e.color,p=e.overrides,d=void 0===p?{}:p,b=h(e,["title","size","color","overrides"]),y=Object(n.d)({component:r.icons&&r.icons.Show?r.icons.Show:null},d&&d.Svg?Object(n.f)(d.Svg):{});return o.createElement(c.a,f({viewBox:"0 0 20 20",ref:t,title:a,size:l,color:u,overrides:{Svg:y}},b),o.createElement("path",{d:"M.2 10a11 11 0 0119.6 0A11 11 0 01.2 10zm9.8 4a4 4 0 100-8 4 4 0 000 8zm0-2a2 2 0 110-4 2 2 0 010 4z"}))}));var v=r(46);function m(e){return(m="function"===typeof Symbol&&"symbol"===typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"===typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e})(e)}function O(){return(O=Object.assign||function(e){for(var t=1;t<arguments.length;t++){var r=arguments[t];for(var o in r)Object.prototype.hasOwnProperty.call(r,o)&&(e[o]=r[o])}return e}).apply(this,arguments)}function w(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){if(!(Symbol.iterator in Object(e))&&"[object Arguments]"!==Object.prototype.toString.call(e))return;var r=[],o=!0,n=!1,i=void 0;try{for(var a,l=e[Symbol.iterator]();!(o=(a=l.next()).done)&&(r.push(a.value),!t||r.length!==t);o=!0);}catch(s){n=!0,i=s}finally{try{o||null==l.return||l.return()}finally{if(n)throw i}}return r}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance")}()}function C(e,t){if(!(e instanceof t))throw new TypeError("Cannot call a class as a function")}function F(e,t){for(var r=0;r<t.length;r++){var o=t[r];o.enumerable=o.enumerable||!1,o.configurable=!0,"value"in o&&(o.writable=!0),Object.defineProperty(e,o.key,o)}}function j(e,t){return!t||"object"!==m(t)&&"function"!==typeof t?x(e):t}function k(e){return(k=Object.setPrototypeOf?Object.getPrototypeOf:function(e){return e.__proto__||Object.getPrototypeOf(e)})(e)}function x(e){if(void 0===e)throw new ReferenceError("this hasn't been initialised - super() hasn't been called");return e}function S(e,t){return(S=Object.setPrototypeOf||function(e,t){return e.__proto__=t,e})(e,t)}function P(e,t,r){return t in e?Object.defineProperty(e,t,{value:r,enumerable:!0,configurable:!0,writable:!0}):e[t]=r,e}var T=function(){return null},R=function(e){function t(){var e,r;C(this,t);for(var n=arguments.length,i=new Array(n),a=0;a<n;a++)i[a]=arguments[a];return P(x(r=j(this,(e=k(t)).call.apply(e,[this].concat(i)))),"inputRef",r.props.inputRef||o.createRef()),P(x(r),"state",{isFocused:r.props.autoFocus||!1,isMasked:"password"===r.props.type,initialType:r.props.type,isFocusVisibleForClear:!1,isFocusVisibleForMaskToggle:!1}),P(x(r),"onInputKeyDown",(function(e){r.props.clearOnEscape&&"Escape"===e.key&&r.inputRef.current&&(r.clearValue(),e.stopPropagation())})),P(x(r),"onClearIconClick",(function(){r.inputRef.current&&r.clearValue(),r.inputRef.current&&r.inputRef.current.focus()})),P(x(r),"onFocus",(function(e){r.setState({isFocused:!0}),r.props.onFocus(e)})),P(x(r),"onBlur",(function(e){r.setState({isFocused:!1}),r.props.onBlur(e)})),P(x(r),"handleFocusForMaskToggle",(function(e){Object(v.d)(e)&&r.setState({isFocusVisibleForMaskToggle:!0})})),P(x(r),"handleBlurForMaskToggle",(function(e){!1!==r.state.isFocusVisibleForMaskToggle&&r.setState({isFocusVisibleForMaskToggle:!1})})),P(x(r),"handleFocusForClear",(function(e){Object(v.d)(e)&&r.setState({isFocusVisibleForClear:!0})})),P(x(r),"handleBlurForClear",(function(e){!1!==r.state.isFocusVisibleForClear&&r.setState({isFocusVisibleForClear:!1})})),r}var r,s,c;return function(e,t){if("function"!==typeof t&&null!==t)throw new TypeError("Super expression must either be null or a function");e.prototype=Object.create(t&&t.prototype,{constructor:{value:e,writable:!0,configurable:!0}}),t&&S(e,t)}(t,e),r=t,(s=[{key:"componentDidMount",value:function(){var e=this.props,t=e.autoFocus,r=e.clearable;this.inputRef.current&&(t&&this.inputRef.current.focus(),r&&this.inputRef.current.addEventListener("keydown",this.onInputKeyDown))}},{key:"componentWillUnmount",value:function(){this.props.clearable&&this.inputRef.current&&this.inputRef.current.removeEventListener("keydown",this.onInputKeyDown)}},{key:"clearValue",value:function(){var e=this.inputRef.current;if(e){var t=Object.getOwnPropertyDescriptor(this.props.type===i.b.textarea?window.HTMLTextAreaElement.prototype:window.HTMLInputElement.prototype,"value");if(t){var r=t.set;if(r){r.call(e,"");var o=function(e){var t;return"function"===typeof window.Event?t=new window.Event(e,{bubbles:!0,cancelable:!0}):(t=document.createEvent("Event")).initEvent(e,!0,!0),t}("input");e.dispatchEvent(o)}}}}},{key:"getInputType",value:function(){return"password"===this.props.type?this.state.isMasked?"password":"text":this.props.type}},{key:"renderMaskToggle",value:function(){var e,t=this;if("password"!==this.props.type)return null;var r=w(Object(n.c)(this.props.overrides.MaskToggleButton,a.g),2),l=r[0],s=r[1],c=w(Object(n.c)(this.props.overrides.MaskToggleShowIcon,y),2),u=c[0],p=c[1],d=w(Object(n.c)(this.props.overrides.MaskToggleHideIcon,b),2),f=d[0],h=d[1],g=this.state.isMasked?"Show password text":"Hide password text",m=(e={},P(e,i.d.mini,"12px"),P(e,i.d.compact,"16px"),P(e,i.d.default,"20px"),P(e,i.d.large,"24px"),e)[this.props.size];return o.createElement(l,O({$size:this.props.size,$isFocusVisible:this.state.isFocusVisibleForMaskToggle,"aria-label":g,onClick:function(){return t.setState((function(e){return{isMasked:!e.isMasked}}))},title:g,type:"button"},s,{onFocus:Object(v.b)(s,this.handleFocusForMaskToggle),onBlur:Object(v.a)(s,this.handleBlurForMaskToggle)}),this.state.isMasked?o.createElement(u,O({size:m,title:g},p)):o.createElement(f,O({size:m,title:g},h)))}},{key:"renderClear",value:function(){var e,t=this,r=this.props,s=r.clearable,c=r.value,u=r.disabled,p=r.overrides,d=void 0===p?{}:p;if(!s||!c||!c.length||u)return null;var b=w(Object(n.c)(d.ClearIconContainer,a.f),2),f=b[0],h=b[1],g=w(Object(n.c)(d.ClearIcon,a.e),2),y=g[0],m=g[1],C="Clear value",F=Object(l.a)(this.props,this.state),j=(e={},P(e,i.d.mini,"14px"),P(e,i.d.compact,"20px"),P(e,i.d.default,"26px"),P(e,i.d.large,"32px"),e)[this.props.size];return o.createElement(f,O({$alignTop:this.props.type===i.b.textarea},F,h),o.createElement(y,O({size:j,tabIndex:0,title:C,"aria-label":C,onClick:this.onClearIconClick,onKeyDown:function(e){!e.key||"Enter"!==e.key&&" "!==e.key||(e.preventDefault(),t.onClearIconClick())},role:"button",$isFocusVisible:this.state.isFocusVisibleForClear},F,m,{onFocus:Object(v.b)(m,this.handleFocusForClear),onBlur:Object(v.a)(m,this.handleBlurForClear)})))}},{key:"render",value:function(){var e=this.props,r=e.value,s=e.type,c=e.overrides,u=c.InputContainer,p=c.Input,d=c.Before,b=c.After,f="password"===this.state.initialType&&this.props.autoComplete===t.defaultProps.autoComplete?"new-password":this.props.autoComplete,h=Object(l.a)(this.props,this.state),g=w(Object(n.c)(u,a.b),2),y=g[0],v=g[1],m=w(Object(n.c)(p,a.a),2),C=m[0],F=m[1],j=w(Object(n.c)(d,T),2),k=j[0],x=j[1],S=w(Object(n.c)(b,T),2),P=S[0],R=S[1];return o.createElement(y,O({"data-baseweb":this.props["data-baseweb"]||"base-input"},h,v),o.createElement(k,O({},h,x)),o.createElement(C,O({ref:this.inputRef,"aria-activedescendant":this.props["aria-activedescendant"],"aria-autocomplete":this.props["aria-autocomplete"],"aria-controls":this.props["aria-controls"],"aria-errormessage":this.props["aria-errormessage"],"aria-label":this.props["aria-label"],"aria-labelledby":this.props["aria-labelledby"],"aria-describedby":this.props["aria-describedby"],"aria-invalid":this.props.error,"aria-required":this.props.required,autoComplete:f,disabled:this.props.disabled,id:this.props.id,inputMode:this.props.inputMode,maxLength:this.props.maxLength,name:this.props.name,onBlur:this.onBlur,onChange:this.props.onChange,onFocus:this.onFocus,onKeyDown:this.props.onKeyDown,onKeyPress:this.props.onKeyPress,onKeyUp:this.props.onKeyUp,pattern:this.props.pattern,placeholder:this.props.placeholder,type:this.getInputType(),required:this.props.required,value:this.props.value,min:this.props.min,max:this.props.max,step:this.props.step,rows:this.props.type===i.b.textarea?this.props.rows:null},h,F),s===i.b.textarea?r:null),this.renderClear(),this.renderMaskToggle(),o.createElement(P,O({},h,R)))}}])&&F(r.prototype,s),c&&F(r,c),t}(o.Component);P(R,"defaultProps",{"aria-activedescendant":null,"aria-autocomplete":null,"aria-controls":null,"aria-errormessage":null,"aria-label":null,"aria-labelledby":null,"aria-describedby":null,adjoined:i.a.none,autoComplete:"on",autoFocus:!1,disabled:!1,error:!1,positive:!1,name:"",inputMode:"text",onBlur:function(){},onChange:function(){},onKeyDown:function(){},onKeyPress:function(){},onKeyUp:function(){},onFocus:function(){},onClear:function(){},clearable:!1,clearOnEscape:!0,overrides:{},pattern:null,placeholder:"",required:!1,size:i.d.default,type:"text"});t.a=R}}]);